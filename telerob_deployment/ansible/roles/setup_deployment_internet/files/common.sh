# Get script location
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPOSITORY_ROOT_DIR=$(realpath "${SCRIPT_DIR}/..")
SECRETS_DIR=${REPOSITORY_ROOT_DIR}/secrets

# Nice colored logging functions
info() {
    echo -e "\e[1;32m[INFO] $@\e[0m"
}

warn() {
    echo -e "\e[1;33m[WARNING] $@\e[0m"
}

error() {
    echo -e "\e[1;31m[ERROR] $@\e[0m"
}

debug() {
    if [ "a$VERBOSE" != "a" ]; then
        echo -e "\e[1;34m[DEBUG] $@\e[0m"
    fi
}


# Helper function to execute a command as root
run_as_root() {
    if [ $EUID -ne 0 ]; then
        debug "Running as root: sudo --preserve-env=BOARDID,FAB,BOARDSKU,BOARDREV \"$@\""
        sudo --preserve-env=BOARDID,FAB,BOARDSKU,BOARDREV "$@"
    else
        "$@"
    fi
}

# Helper function to execute a command and log if verbose is set
run() {
    if [ "a$VERBOSE" != "a" ]; then
        debug Running: "$@"
    fi
    "$@"
}

# Get information about the boards
list_boards() {
    local prefix=$1
    local config=$(cat $REPOSITORY_ROOT_DIR/boards/configurations.conf)

    # Trim empty lines and comment lines
    config=$(echo "$config" | sed '/^$/d' | sed '/^#/d')

    # The first column is the board names
    local names=$(echo "$config" | cut -d' ' -f1)
    local escaped_prefix=$(echo "$prefix" | sed 's/\\/\\\\/g')
    local names_with_prefix=$(echo "$names" | sed "s/^/$escaped_prefix/")
    echo -e "$names_with_prefix"
}

get_board_info() {
    local config=$(cat $REPOSITORY_ROOT_DIR/boards/configurations.conf)
    local robot_type=$1

    # Trim empty lines and comment lines
    config=$(echo "$config" | sed '/^$/d' | sed '/^#/d')

    # Find the line with the robot type
    local line=$(echo "$config" | grep -E "^$robot_type ")

    if [ -z "$line" ]; then
        error "Could not find robot type $robot_type in configurations.conf"
        error "Available robot types:"
        list_boards "\e[1;31m[ERROR]  - "
        echo -e "\e[0m"
        exit 1
    fi

    # Set the environment variables
    export JETSON_TYPE=$(echo "$line" | awk '{print $2}')
    export BSP_CONFIG=$(echo "$line" | awk '{print $3}')
    export ROOTFS_ROBOT_TYPE=$(echo "$line" | awk '{print $4}')
    export ROBOT_TYPE=$robot_type

    if [ "$JETSON_TYPE" == "orin-nx" ]; then
        export JETSON_ID=0x23
        export JETSON_SOC=t234
    elif [ "$JETSON_TYPE" == "xavier-nx" ]; then
        export JETSON_ID=0x19
        export JETSON_SOC=t194
    else
        error "Unknown Jetson type $JETSON_TYPE"
        exit 1
    fi

    debug "JETSON_TYPE=$JETSON_TYPE"
    debug "JETSON_ID=$JETSON_ID"
    debug "JETSON_SOC=$JETSON_SOC"
    debug "BSP_CONFIG=$BSP_CONFIG"
    debug "ROOTFS_ROBOT_TYPE=$ROOTFS_ROBOT_TYPE"
    debug "ROBOT_TYPE=$ROBOT_TYPE"
}

# Function to setup the secrets
function secrets_setup() {
    if [ ! -f ${SECRETS_DIR}/fuses-orin.xml ]; then
        info "Decrypting secrets"
        run_as_root ${SCRIPT_DIR}/decrypt-secrets.sh
    else
        info "Secrets already present"
    fi

    if [ "$JETSON_TYPE" == "orin-nx" ]; then
        export KEY_PKC=${SECRETS_DIR}/pkc1-orin.pem
        export KEY_SBK=${SECRETS_DIR}/sbk-orin.key
        export FUSES_CONFIG=${SECRETS_DIR}/fuses-orin.xml
    elif [ "$JETSON_TYPE" == "xavier-nx" ]; then
        export KEY_PKC=${SECRETS_DIR}/pkc1-xavier.pem
        export KEY_SBK=${SECRETS_DIR}/sbk-xavier.key
        export FUSES_CONFIG=${SECRETS_DIR}/fuses-xavier.xml
    else
        error "Unknown Jetson type $JETSON_TYPE"
        exit 1
    fi
    debug "KEY_PKC=$KEY_PKC"
    debug "KEY_SBK=$KEY_SBK"
}
