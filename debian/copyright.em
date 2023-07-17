Format: Bloom subset of https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: @(Name)
@[if BugTracker]Upstream-Contact: @(BugTracker)@\n@[end if]@
@[if Source]Source: @(Source)@\n@[end if]@
@[for License, Text in Licenses]@

Files: See file headers in repository for details
Copyright: See package copyright in source code for details
License: @(License)
 @(Text)
@[end for]@
