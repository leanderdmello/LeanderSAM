# Configuration Validation

This directory contains the configuration file validation system for Telerob deployments.

## Files

- `validate-configuration-file.py` - Python script that validates TOML configuration files
- `configuration-schema.toml` - Schema file defining required fields and their types
- `update-configuration-file.sh` - Shell script that validates and copies configuration from SD card

## Usage

### Validating a Configuration File

```bash
# Using the default schema file
python3 validate-configuration-file.py config.toml configuration-schema.toml

# Using default hardcoded schema (fallback)
python3 validate-configuration-file.py config.toml
```

## Modifying Required Fields

To add, remove, or modify required fields, edit `configuration-schema.toml`:

### Schema Format

```toml
["field.path"]
type = "str" | "int" | "float" | "bool"
required = true | false
non_empty = true | false  # Only for string fields
```

### Examples

#### Adding a Required String Field

```toml
["network.gateway"]
type = "str"
required = true
non_empty = true
# This requires config["network"]["gateway"] to be a non-empty string
```

#### Adding an Optional Integer Field

```toml
["stream.framerate"]
type = "int"
required = false
non_empty = false
# This validates the type if present, but doesn't require it
```

#### Adding a Nested Field

```toml
["stream.highres.width"]
type = "int"
required = true
non_empty = false
# This requires config["stream"]["highres"]["width"] to be an integer
```

### Field Types

- `"str"` - String value
- `"int"` - Integer value
- `"bool"` - Boolean value (true/false)
- `"float"` - Floating point number

### Field Options

- `required` - If `true`, the field must be present in the configuration
- `non_empty` - If `true` (only for strings), the field cannot be an empty string

## Current Required Fields

The current schema requires:

- `network.address` (string, non-empty) - Used by process-configuration-file.sh for network configuration

## Integration

The `update-configuration-file.sh` script automatically uses the schema file if it exists in the same directory. This ensures that configuration files from SD cards are validated before being copied to the deployment directory.
