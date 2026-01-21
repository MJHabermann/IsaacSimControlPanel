#!/bin/bash
# Show network information for accessing HMI from other devices

echo "╔══════════════════════════════════════════════════════════╗"
echo "║         Network Information for HMI Access               ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# WSL IP
WSL_IP=$(hostname -I | awk '{print $1}')
echo "WSL IP Address: $WSL_IP"
echo ""

echo "══════════════════════════════════════════════════════════"
echo "To access from your tablet, run these commands in"
echo "Windows PowerShell (as Administrator):"
echo "══════════════════════════════════════════════════════════"
echo ""
echo "1. Set up port forwarding:"
echo "   netsh interface portproxy add v4tov4 listenport=5000 listenaddress=0.0.0.0 connectport=5000 connectaddress=$WSL_IP"
echo ""
echo "2. Open firewall (only needed once):"
echo "   netsh advfirewall firewall add rule name=\"Flask HMI\" dir=in action=allow protocol=tcp localport=5000"
echo ""
echo "3. Then access from tablet at:"
echo "   http://<YOUR_WINDOWS_IP>:5000"
echo ""
echo "   Find Windows IP with: ipconfig (look for IPv4 Address)"
echo ""
echo "══════════════════════════════════════════════════════════"
echo "To remove port forwarding later:"
echo "   netsh interface portproxy delete v4tov4 listenport=5000 listenaddress=0.0.0.0"
echo "══════════════════════════════════════════════════════════"
