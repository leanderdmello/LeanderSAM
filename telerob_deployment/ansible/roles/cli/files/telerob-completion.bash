#!/bin/bash

_telerob_completion() {
    local cur prev words cword
    _init_completion || return

    # If we're completing the first argument (the main command)
    if [ $cword -eq 1 ]; then
        COMPREPLY=($(compgen -W "reload-config compose" -- "$cur"))
        return
    fi

    if [ "${words[1]}" = "enter" ] || [ "${words[1]}" = "rebuild" ]; then
        COMPREPLY=($(compgen -W "slam interface" -- "$cur"))
        return
    fi

    # If the first argument is "compose", forward to docker compose completion
    if [ "${words[1]}" = "compose" ]; then
        # Shift arguments to mimic "docker compose ..."
        local new_words=("docker" "compose" "${words[@]:2}")
        local new_cword=$((cword - 1))

        # Call docker's completion function if available
        if declare -F _docker > /dev/null 2>&1; then
            # Temporarily replace COMP_WORDS and COMP_CWORD
            local old_words=("${COMP_WORDS[@]}")
            local old_cword=$COMP_CWORD

            COMP_WORDS=("${new_words[@]}")
            COMP_CWORD=$new_cword

            _docker

            # Restore original values
            COMP_WORDS=("${old_words[@]}")
            COMP_CWORD=$old_cword
        else
            # Fallback: provide common compose commands
            if [ $new_cword -eq 2 ]; then
                COMPREPLY=($(compgen -W "build config cp create down events exec images kill logs ls pause port ps pull push restart rm run start stop top unpause up version" -- "$cur"))
            fi
        fi

        return
    fi
}

complete -F _telerob_completion telerob
