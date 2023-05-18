#!/bin/bash
# Carrier detecting
reg_err='^(169\.254)'
reg_ipv4='([0-9]{1,3}\.){3}[0-9]{1,3}'
printf "%s" "Carrier connecting..."
while ! ip addr show dev $1 | grep -P 'inet(?!\d+)' | grep -Po $reg_ipv4 &> /dev/null
do
    printf "%c" "."
done
printf "\t%s\n" "Carrier detected."

# Network detecting
printf "%s" "Network connecting..."
while ip addr show dev $1 | grep -P 'inet(?!\d+)' | grep -Po $reg_ipv4 | grep -P $reg_err &> /dev/null
do
    printf "%c" "."
done
printf "\t%s\n" "Network detected."

# Process here
