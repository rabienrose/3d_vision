#!/bin/bash

root_dir=$1

config_yaml=${root_dir}/pc_match_config.yaml
rosparam load ${config_yaml}
./devel/lib/pcdesc_matcher/pcdesc_matcher ${root_dir} loc

