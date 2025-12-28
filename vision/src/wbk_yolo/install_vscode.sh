#!/bin/bash
# Install VS Code on Ubuntu/Linux (for Jetson Nano)

echo "Installing VS Code..."

# Method 1: Using snap (easiest, if snap is available)
if command -v snap &> /dev/null; then
    echo "Installing via snap..."
    sudo snap install code --classic
    echo "✓ VS Code installed via snap"
    exit 0
fi

# Method 2: Using apt repository (recommended for Jetson)
echo "Installing via apt repository..."

# Add Microsoft GPG key
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

# Update and install
sudo apt update
sudo apt install -y code

# Cleanup
rm -f packages.microsoft.gpg

echo "✓ VS Code installed via apt"
echo ""
echo "To launch VS Code, run: code"


