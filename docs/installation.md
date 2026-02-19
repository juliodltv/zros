# Installation

## 1. Setting `UV`, the Python project manager

To facilitate the creation of virtual environments and manage Python packages and their dependencies we use a state of the art framework [uv](https://docs.astral.sh/uv/), its installation is straightforward and can be done via the following command:

=== "macOS/Linux"
    Using `curl`
    ```bash
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```
    Using `wget`
    ```bash
    wget -qO- https://astral.sh/uv/install.sh | sh
    ```
=== "Windows"
    Use `irm` to download the script and execute it with `iex`:
    ```powershell
    powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
    ```

## 2. Install ZROS

=== "From PyPI"
    !!! abstract "Recommended for most users"
        Install the latest stable release from the Python Package Index (PyPI).

    ```bash
    uv venv
    uv pip install zros
    ```

    !!! info "System Requirements"
    ZROS requires Python 3.8 or higher. If you have a previous version, you can create a virtual environment with a compatible version using `uv venv --python 3.8` or higher.

=== "From Source"
    !!! abstract "Recommended for developers"
        Install the latest development version directly from the GitHub repository.

    ```bash
    git clone https://github.com/juliodltv/zros.git
    cd zros
    uv sync
    ```