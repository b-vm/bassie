#!/bin/bash

# move files
mkdir bassie_urdf
cp sld_export/bassie.SLDASM/urdf/bassie.SLDASM.urdf bassie_urdf/bassie.urdf
cp -r sld_export/bassie.SLDASM/meshes bassie_urdf/
cp bassie_urdf/bassie.urdf bassie_urdf/meshes/bassie.xml

# fix mesh paths
sed -i 's/package:\/\/bassie\.SLDASM\/meshes\//meshes\//g' "bassie_urdf/bassie.urdf"

echo "Done!"
