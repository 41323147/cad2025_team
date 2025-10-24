#!/bin/bash

members=("41323109" "41323040" "41171130")

for member in "${members[@]}"; do
    echo "設定成員 $member 的分支關聯..."
    git branch --set-upstream-to=origin/feature/$member feature/$member
done
