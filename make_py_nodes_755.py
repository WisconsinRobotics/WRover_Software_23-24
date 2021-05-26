#!/usr/bin/env python3

import os

for pkg_name in os.listdir('src'):
    pkg_dir = f'src/{pkg_name}'
    if os.path.isdir(pkg_dir):
        nodes_dir = f'{pkg_dir}/nodes'
        if os.path.isdir(nodes_dir):
            for node_name in os.listdir(nodes_dir):
                node_path = f'{nodes_dir}/{node_name}'
                print(node_path)
                os.chmod(node_path, 755)
