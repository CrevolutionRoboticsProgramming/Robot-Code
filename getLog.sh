#!/bin/bash 

scp lvuser@roborio-2851-FRC.local:/home/lvuser/log.bag log.bag
ssh lvuser@roborio-2851-FRC.local 'rm -f log.bag'
badlogvis log.bag log.html
rm -f log.bag
