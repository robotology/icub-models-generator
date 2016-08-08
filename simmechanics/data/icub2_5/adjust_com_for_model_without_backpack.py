#!/usr/bin/env python

import os
import argparse
from lxml import etree


parser = argparse.ArgumentParser(description='Update the COM  of the chest to convert a iCub 2.5 with backpack to a model without backpack.')
parser.add_argument('--input_urdf', dest='input_urdf', nargs='?', action='store', help='Input file (URDF model extracted from the SimMechanics Model)')
parser.add_argument('--output_urdf', dest='output_urdf', nargs='?', action='store', help='Output file, with the COM udpdated.')

args = parser.parse_args();

tree = etree.parse(args.input_urdf);
robot = tree.getroot();
for link in robot.getiterator("link"):
   if( link.get('name') == "chest" ): 
       link_chest = link;

for inertial in link_chest.getiterator('inertial'):
    for origin in inertial.getiterator('origin'):
        origin_chest = origin;

if( origin_chest.get('xyz') != '0.000188619190735 0.073091 -0.047238'):
    print("Warning: unexpected com for the chest ",origin_chest.get('xyz'));
    
origin_chest.set('xyz','0.000188619190735 0.023091 -0.013238');
    
# Save to file (see http://stackoverflow.com/questions/12517451/python-automatically-creating-directories-with-file-output)
if not os.path.exists(os.path.dirname(args.output_urdf)):
    try:
        os.makedirs(os.path.dirname(args.output_urdf))
    except OSError as exc: # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

with open(args.output_urdf, "w") as text_file:
    text_file.write(etree.tostring(robot));
    

    
       
   
