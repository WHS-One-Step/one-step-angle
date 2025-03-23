# Written by: Christopher Gholmieh
# Sources:

# Functions:
function bootstrap-environment() {
    # Validation:
    if [ ! -f ./main.py ]; then
        echo "[!] Please rerun this script in the main directory."

        exit
    fi

    # Initialization:
    echo "[*] Initializing virtual environment.."

    # Command:
    python3 -m venv venv

    # Libraries:
    echo "[*] Installing appropriate dependencies.."

    # Command:
    ./venv/bin/pip3 install -r ./requirements.txt

    # Exiting:
    echo "[*] Finished  up environment with dependencies.." && exit
}


# Main:
bootstrap-environment
