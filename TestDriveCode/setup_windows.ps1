if (Test-Path -Path '.\venv') {
    .\venv\Scripts\activate
}
else {
    python3.10.exe -m venv venv
    .\venv\Scripts\activate;
    python3.10.exe -m pip install --upgrade pip setuptools;
    python3.10.exe -m pip install pyyaml
    python3.10.exe -m pip install ./mechae263C_helpers --no-warn-script-location;
    python3.10.exe -m pip install ./dynamixel-controller --no-warn-script-location;
}