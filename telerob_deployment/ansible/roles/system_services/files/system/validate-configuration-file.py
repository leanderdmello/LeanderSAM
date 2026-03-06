#!/usr/bin/env python3
"""
Validate a TOML configuration file for Telerob deployment.

This script validates that:
1. The file is valid TOML syntax
2. Required fields are present
3. Field values are of expected types

Required fields and their types are defined in a schema file.
By default, uses configuration-schema.toml in the same directory.
"""

import sys
import toml
from pathlib import Path


# Default schema defining required fields and their types
# This can be overridden by providing a schema file
DEFAULT_SCHEMA = {
    "network.address": {"type": str, "required": True, "non_empty": True},
}


def load_schema(schema_path):
    """
    Load validation schema from a TOML file.

    Schema format:
    [field_name]
    type = "str" | "int" | "float" | "bool"
    required = true | false
    non_empty = true | false  # Only for strings

    Args:
        schema_path: Path to the schema file

    Returns:
        dict: Schema dictionary

    Raises:
        FileNotFoundError: If schema file doesn't exist
        tomllib.TOMLDecodeError: If schema file has invalid TOML syntax
    """
    if not Path(schema_path).is_file():
        raise FileNotFoundError(f"Schema file not found: {schema_path}")

    schema = {}

    with open(schema_path, "r") as f:
        schema_config = toml.load(f)

    type_mapping = {
        "str": str,
        "int": int,
        "float": float,
        "bool": bool,
    }

    for field_name, field_spec in schema_config.items():
        schema[field_name] = {
            "type": type_mapping.get(field_spec.get("type", "str"), str),
            "required": field_spec.get("required", False),
            "non_empty": field_spec.get("non_empty", False),
        }

    return schema


def get_nested_value(config, path):
    """
    Get a value from nested dictionary using dot notation.

    Args:
        config: Configuration dictionary
        path: Dot-separated path (e.g., "network.address")

    Returns:
        tuple: (found, value)
    """
    keys = path.split(".")
    current = config

    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return False, None
        current = current[key]

    return True, current


def validate_configuration(config_path, schema=None):
    """
    Validate a Telerob configuration file.

    Args:
        config_path: Path to the configuration file
        schema: Optional schema dict. If None, uses DEFAULT_SCHEMA

    Returns:
        tuple: (is_valid, error_message)
    """
    if schema is None:
        schema = DEFAULT_SCHEMA

    try:
        # Check if file exists
        if not Path(config_path).is_file():
            return False, f"Configuration file not found: {config_path}"

        # Parse TOML file
        with open(config_path, "r") as f:
            config = toml.load(f)

        # Validate each field in schema
        for field_path, field_spec in schema.items():
            found, value = get_nested_value(config, field_path)

            # Check if required field is present
            if field_spec.get("required", False) and not found:
                # Provide helpful error message
                keys = field_path.split(".")
                if len(keys) > 1:
                    section = keys[0]
                    if section not in config:
                        return False, f"Missing required section: [{section}]"
                return False, f"Missing required field: {field_path}"

            # If field is present, validate its type and value
            if found:
                expected_type = field_spec.get("type", str)
                if not isinstance(value, expected_type):
                    type_name = expected_type.__name__
                    return False, f"Field {field_path} must be a {type_name}"

                # Check non-empty constraint for strings
                if field_spec.get("non_empty", False) and isinstance(value, str):
                    if not value.strip():
                        return False, f"Field {field_path} cannot be empty"

        # All validations passed
        return True, "Configuration file is valid"

    except toml.TomlDecodeError as e:
        return False, f"Invalid TOML syntax: {e}"
    except Exception as e:
        return False, f"Validation error: {e}"


if __name__ == "__main__":
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print(
            "Usage: validate-configuration-file.py <config_file> [schema_file]",
            file=sys.stderr,
        )
        print("", file=sys.stderr)
        print("If schema_file is not provided, uses default schema.", file=sys.stderr)
        print(
            "Schema file should be a TOML file with field definitions.", file=sys.stderr
        )
        sys.exit(1)

    config_file = sys.argv[1]
    schema = None

    # Load schema if provided
    if len(sys.argv) == 3:
        schema_file = sys.argv[2]
        try:
            schema = load_schema(schema_file)
        except Exception as e:
            print(
                f"ERROR: Failed to load schema from {schema_file}: {e}", file=sys.stderr
            )
            sys.exit(1)

    is_valid, message = validate_configuration(config_file, schema)

    if is_valid:
        print(message)
        sys.exit(0)
    else:
        print(f"ERROR: {message}", file=sys.stderr)
        sys.exit(1)
