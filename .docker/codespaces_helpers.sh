#
# GitHub Codespaces helpers for ScenarIO and gym-ignition
#

function build_cmake() {
    cd $CODESPACE_VSCODE_FOLDER
    cmake -S . -B build/ -GNinja -DCMAKE_BUILD_TYPE=Debug || return 1
    cmake --build build/ || return 1
}

function install_cmake() {
    cd $CODESPACE_VSCODE_FOLDER
    sudo cmake --install build/ || return 1
}

function install_python() {
    cd $CODESPACE_VSCODE_FOLDER
    pip install -e scenario/ || return 1
    pip install -e .[all] || return 1
}

function developer_setup() {
    build_cmake || return 1
    install_cmake || return 1
    install_python || return 1
}

function uninstall_cmake() {
    cd $CODESPACE_VSCODE_FOLDER/build
    sudo ninja uninstall || return 1
}

function uninstall_python() {
    pip uninstall -y scenario gym-ignition || return 1
}

function developer_uninstall() {
    uninstall_cmake || return 1
    uninstall_python || return 1
}

function developer_cleanup() {
    developer_uninstall || return 1
    rm -rf $CODESPACE_VSCODE_FOLDER/build || return 1
}
