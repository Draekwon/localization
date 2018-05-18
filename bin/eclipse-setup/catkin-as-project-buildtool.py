#!/usr/bin/env python

import xml.etree.ElementTree

tree = xml.etree.ElementTree.parse('.project')
root = tree.getroot()

changes = { 'org.eclipse.cdt.make.core.build.command' : '/bin/bash',
            'org.eclipse.cdt.make.core.build.target.inc' : '-c "catkin build --force-color --status-rate 1 `basename ${project_loc}`"',
            'org.eclipse.cdt.make.core.build.target.clean' : '-c "catkin --force-color clean `basename ${project_loc}`;${workspace_loc:/}/bin/make-eclipse-projects.sh `basename ${project_loc}`"',
            'org.eclipse.cdt.make.core.build.location' : '${workspace_loc:/}',
            'org.eclipse.cdt.make.core.build.arguments' : '',
            'org.eclipse.cdt.make.core.enableAutoBuild' : 'false',
            'org.eclipse.cdt.make.core.enableCleanBuild' : 'true',
            'org.eclipse.cdt.make.core.enableFullBuild' : 'false',
            'org.eclipse.cdt.make.core.enabledIncrementalBuild' : 'true'
}

for dic in root.iter('dictionary'):
    if(len(list(dic)) > 1):
        pair = list(dic)

        if pair[0].text in changes:
            pair[1].text = changes[pair[0].text]


tree.write('.project')
