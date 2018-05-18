#!/usr/bin/env python

import xml.etree.ElementTree

tree = xml.etree.ElementTree.parse('.project')
root = tree.getroot()

for child in root:
    if child.tag == 'name':
        if child.text.find("-Debug@") > 0:
            child.text = child.text[0:child.text.find("-Debug@")]
        if child.text.find("-Release@") > 0:
            child.text = child.text[0:child.text.find("-Release@")]
        break

tree.write('.project')
