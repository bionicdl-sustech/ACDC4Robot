# ACDC4Robot Tools
This folder contains tools that help users to use ACDC4Robot more convenient.
The usage for each tools described below.

## FlattenOcc
Since ACDC4Robot currently does not support nested occurrence, user needs to transfrom nested occurrence into a single occurrence that contains all the geometry bodies in the nested child occurrences. 
In simple terms, it means to cut and paste bodies of child occurrences into the target occurrence, and then delete those child occurrences.
So we provide a tool called FlattenOcc to automatically produce this process.

User just run this script and choose the occurrence that you want to flatten.