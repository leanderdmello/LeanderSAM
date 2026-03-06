// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <exception>
#include <string>

#include <nlohmann/json.hpp>

class JSONRPCException : std::exception
{
public:
    JSONRPCException(int code, std::string message) : message_(message), error_code_(code) {}

    const char *what() const noexcept override { return message_.c_str(); }

    int code() const { return error_code_; }

private:
    std::string message_{};
    int         error_code_{};
};

class NotImplementedException : public JSONRPCException
{
public:
    NotImplementedException() : JSONRPCException(0, "The method is not yet implemented") {}
};

class InvalidParameterValueException : public JSONRPCException
{
public:
    InvalidParameterValueException(const std::string &parameter_name,
                                   const std::string &specification = "")
        : JSONRPCException(
              1,
              "The value of parameter " + parameter_name +
                  " is not within the expected value range, according to the specification: " +
                  specification)
    {
    }

    InvalidParameterValueException(const nlohmann::json::exception &exception)
        : JSONRPCException(1, exception.what())
    {
    }
};

class NotFoundException : public JSONRPCException
{
public:
    NotFoundException()
        : JSONRPCException(2, "No result could be found given the provided search parameters")
    {
    }
};

class NotAllowedException : public JSONRPCException
{
public:
    NotAllowedException(const std::string &reason = "")
        : JSONRPCException(
              3,
              "The command is not allowed in the current state of the Module: " + reason)
    {
    }
};

class ErrorProcessingRequestException : public JSONRPCException
{
public:
    ErrorProcessingRequestException(const std::string &message = "")
        : JSONRPCException(4, "An error occurred while processing the request: " + message)
    {
    }
};

class MethodNotFoundException : public JSONRPCException
{
public:
    MethodNotFoundException(const std::string &message = "")
        : JSONRPCException(-32601, "The requested rpc method is not supported: " + message)
    {
    }
};
