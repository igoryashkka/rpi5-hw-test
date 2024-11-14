#!/bin/bash

# Define the virtual environment name and repository URL
VENV_NAME="test-hw"
REPO_URL="https://github.com/igoryashkka/rpi5-hw-test.git"
REPO_DIR="rpi5-hw-test"

# Check if Python3 and virtualenv are installed
if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed. Please install it and try again."
    exit 1
fi

if ! python3 -m venv --help &> /dev/null; then
    echo "virtualenv module is not installed. Installing it..."
    python3 -m pip install --user virtualenv
fi

# Step 1: Create virtual environment
if [ ! -d "$VENV_NAME" ]; then
    echo "Creating virtual environment '$VENV_NAME'..."
    python3 -m venv "$VENV_NAME"
    echo "Virtual environment '$VENV_NAME' created."
else
    echo "Virtual environment '$VENV_NAME' already exists."
fi

# Step 2: Activate the virtual environment and install requirements
source "$VENV_NAME/bin/activate"
echo "Virtual environment '$VENV_NAME' activated."

# Install necessary libraries
echo "Installing required libraries..."
python3 -m pip install --upgrade pip
python3 -m pip install smbus2 smbus numpy easydict imusensor pymavlink bmp280

# Install libcamera-apps
echo "Installing libcamera-apps..."
sudo apt update
sudo apt install -y libcamera-apps

# Step 3: Clone the GitHub repository if not already cloned
if [ ! -d "$REPO_DIR" ]; then
    echo "Cloning repository from $REPO_URL..."
    git clone "$REPO_URL"
else
    echo "Repository '$REPO_DIR' already exists."
fi

# Deactivate virtual environment
deactivate
echo "Setup complete and virtual environment deactivated."

