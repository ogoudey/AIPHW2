#!/usr/bin/env bash



if [ -d ".venv" ]; then
    echo "Activating environment requirements..."
    source .venv/bin/activate
else
    echo "Would you like to set up the Python environment? [hit enter to continue or Ctrl+C to cancel]"
    read
    echo "Creating virtual environment..."
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
fi
echo "Setup complete. Running submission..."

python3 space.py