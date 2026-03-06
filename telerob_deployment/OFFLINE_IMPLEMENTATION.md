# Implementation Summary: Offline Package Installation

## Changes Made

### 1. Docker Package Downloader (`docker/package-downloader.Dockerfile`)
- New Dockerfile that runs on `l4t-base:r36.2.0` (target architecture)
- Downloads all required apt packages to `/packages/apt/`
- Downloads all required Python wheels to `/packages/pip/`
- Can be run during deployment archive build process

### 2. Makefile Updates (`telerob_deployment/Makefile`)
- Added `system_setup_role_build_dir` variable for system_setup role build directory
- Updated `packages_cache_dir` to point to system_setup role: `$(system_setup_role_build_dir)/packages-offline`
- Added `check-package-deps` phony target for dependency tracking
- Added `$(build_dir)/package-deps.md5` target that:
  - Tracks hash of `docker/package-downloader.Dockerfile`
  - Invalidates cache when Dockerfile changes
- Split package download into two targets:
  - `$(build_dir)/package-downloader.built` - builds the Docker image
  - `$(packages_cache_dir)/downloaded` - extracts packages from image
- Integrated package download into build chain:
  - Now a dependency of `$(ansible_dir)/built` target
  - Automatically runs when building archive if needed
  - Only rebuilds when Dockerfile changes

### 3. Ansible Playbook Updates (`ansible/roles/system_setup/tasks/main.yml`)
Significantly simplified the system_setup role:

**Setup Phase (conditional based on internet_access):**
- **Online:** `Update apt cache (online mode)` - runs `apt-get update` from standard repositories
- **Offline:** `Set up offline apt repository` - creates and configures local apt repository
  - Copies `.deb` files from `roles/system_setup/files/build/packages-offline/apt/` to `/var/local/offline-repo/`
  - Generates repository index using `apt-ftparchive`
  - Configures apt to use local repository via `/etc/apt/sources.list.d/offline-local.list`
  - Runs `apt-get update` to index local repository
- **Offline:** `Set up offline pip directory` - copies Python wheels to `/opt/offline-packages/pip/`

**Installation Phase (unified for both online and offline!):**
- `Install gpiod for LED controller` - single task for both modes
- `Install evtest for sleep on short service` - single task for both modes
- `Install socat for udp forwarder` - single task for both modes
- `Install ansible and dependencies` - single task for both modes
- `Install toml parsing libraries` - only Python packages differ (online vs offline pip flags)

**Key Improvements:**
- Removed `inventory_hostname == 'remote'` condition (no longer needed with bundled packages)
- Removed duplicate tasks - same `apt-get install` commands work for both modes
- Only Python packages need separate online/offline handling (pip flags)
- Much cleaner and more maintainable code

**Note:** The `internet_access` variable is automatically calculated during deployment based on network connectivity checks performed in a previous step.

### 4. Documentation (`telerob_deployment/OFFLINE_INSTALLATION.md`)
Comprehensive guide covering:
- Architecture and flow
- Supported packages
- Usage instructions
- How it works (both online and offline modes)
- Troubleshooting guide
- Customization instructions
- Performance notes

## How It Works

### Build Process
```
make archive
├── check-package-deps (automatic)
│   └── Check if docker/package-downloader.Dockerfile changed
├── make download-packages (automatic, if needed)
│   ├── Build telerob-package-downloader:latest (arm64)
│   │   └── Triggered only if Dockerfile changed
│   ├── Extract /packages/apt/*.deb
│   ├── Extract /packages/pip/*.whl
│   └── Store in $(system_setup_role_build_dir)/packages-offline/
├── Ansible roles with packages
└── Final archive includes packages-offline/ in system_setup role
```

### Deployment (Offline Mode)
```
Deployment starts (internet_access auto-detected or manually set to false)
├── Copy packages to /var/local/offline-repo/
├── Run apt-ftparchive to create repository index
├── Configure /etc/apt/sources.list.d/offline-local.list
├── Run apt-get update (indexes local repo)
├── Copy pip wheels to /opt/offline-packages/pip/
└── Install all packages using standard commands
    ├── apt: apt-get install (uses local repo automatically)
    └── pip: pip install --no-index --find-links=/opt/offline-packages/pip
```

### Deployment (Online Mode)
```
Deployment starts (internet_access=true or auto-detected as online)
├── Skip offline setup tasks
└── Use standard apt-get and pip install (requires internet)
```

## Integration Points

### Makefile Integration
The package download is now part of the normal build process:
- Runs automatically when needed (triggered by Dockerfile changes)
- Caches results (uses MD5 hash of Dockerfile for invalidation)
- Split into separate steps:
  - Image build: `$(build_dir)/package-downloader.built`
  - Package extraction: `$(packages_cache_dir)/downloaded`
- Can be run manually: `make download-packages`
- Packages stored in system_setup role's files/build directory

### Ansible Integration
The playbook uses Ansible's conditional logic (`when` clauses):
- `internet_access == true` → online installation
- `internet_access == false` → offline installation
- Both paths produce the same result

### Deployment Role Integration
Packages are stored in `roles/system_setup/files/build/packages-offline/`:
- Part of system_setup role's file tree (co-located with where they're used)
- Automatically included in deployment archive
- Ansible playbook references them with relative paths from the role's files directory

## Testing the Implementation

### Test Package Download
```bash
cd telerob_deployment
make clean
make download-packages
ls -lh build/flash_archive/ansible/roles/system_setup/files/build/packages-offline/
```

### Test Archive Building
```bash
make archive
tar -tzf build/telerob_flash_archive.prod.tar.gz | grep packages-offline
```

### Test Offline Deployment
```bash
# On Jetson with extracted archive
ansible-playbook -i inventory.yml -e internet_access=false deploy.yml
```

### Test Online Deployment
```bash
# On Jetson with internet connectivity (auto-detected)
ansible-playbook -i inventory.yml deploy.yml
```

### Test Dependency Tracking
```bash
# Modify docker/package-downloader.Dockerfile
make archive  # Should rebuild packages

# Run again without changes
make archive  # Should skip package rebuild (cached)
```

## Advantages of This Approach

1. **Target Architecture:** Downloads on correct ARM64 arch, avoiding cross-compilation issues
2. **Automatic:** Integrated into standard build process via Makefile
3. **Efficient:** Only rebuilds packages when Dockerfile changes (dependency tracking)
4. **Modular:** Packages stored in system_setup role where they're used
5. **Split Targets:** Build image and extract packages are separate, cacheable steps
6. **Fallback:** Works online or offline transparently (auto-detected)
7. **Unified Code Path:** Same installation commands work for both online and offline
8. **No Duplication:** Single set of installation tasks, no conditional variants needed
9. **Easy to Extend:** Easy to add new packages by editing Dockerfile
10. **Self-contained:** Packages bundled with deployment archive
11. **Deterministic:** Always the same versions due to package pinning
12. **Proper Dependency Resolution:** Uses apt repository instead of dpkg, handles dependencies correctly

## Limitations and Future Improvements

### Current Limitations
- Apt dependencies aren't recursively resolved (manual in Dockerfile)
- Python dependency resolution uses `--no-deps` (must list all)
- Error handling silently ignores missing packages
- No integrity verification of cached packages

### Potential Enhancements
- Add package checksums for verification
- Implement incremental updates
- Auto-generate Dockerfile from package list
- Add pre-deployment validation
- Support for custom package repositories
- Compression of large package sets
