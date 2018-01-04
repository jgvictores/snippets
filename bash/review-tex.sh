#!/bin/bash

# Punctuation
grep "\.\." *.tex | grep -v "\.\.\." | grep -v "\.\.\/"  # sed: \.

# 
grep "This way" *.tex  # sed: omit
grep "But" *.tex  # sed: omit or replace by however

#
grep -i "do" *.tex # sed: perform
grep -i "also" *.tex # sed: additionally
grep -i "just" *.tex # sed: only

# 
grep -i "problem" *.tex # sed: issue, challenge

