# Why there are no launch files

Launch files, written in python or in xml, will open all the nodes concurrently as different processes. The tests are set to expire after x amount of messages sent. Therefore they will stop spinning on their own.

An approach was tried to run them as a python script, however it was unsucessfull to get the node to destroy itself and open a new one.