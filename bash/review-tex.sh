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
grep -i "get" *.tex # sed: obtain
grep -i "like" *.tex # sed: such as

#
grep -i "implementation" *.tex  # sed: omit (unless you're in the Experimental section)
