#!/bin/bash
# usage: ./push.sh x 30 0.3    (axis, force [N], duration [s])
E='name: "aliengo::base", type: LINK'
gz topic -t /world/default/wrench/persistent -m gz.msgs.EntityWrench \
  -p "entity: {$E}, wrench: {force: {$1: $2}}"
sleep $3
gz topic -t /world/default/wrench/clear -m gz.msgs.Entity -p "$E"
