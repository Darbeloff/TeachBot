Written by: Tongxi Yan
Date: 8/12/2020

Files are now organized by robot models. Due to the hardware difference between Sawyer and UR, some instructions cannot be exactly the same.

In addition to separating files by robot name, one can also add if-else statements in json instructions that reads what robot is currently running. I thought that this method is somehow cumbersome and adding more files didn't really make the overall structure complicated.

For audio and text, I also changed corresponding directory paths in Module.js. Anyone making changes to the file's organizations should also modify paths to avoid any bugs.
