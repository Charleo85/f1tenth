# F1/10 Team 1

Team Blog for F1/10 Team 1

Our progress updates are in the [Wiki](https://github.com/Charleo85/f1tenth/wiki).

## Milestones

- Car Assembly
- Keyboard control

## Notes

#### Mapping Files into Jetson

Use the following commands to map files from this repo into Jetson:
```shell
$ scp -r ./race/ ubuntu@192.168.40.1:~/catkin_ws/src/f1tenth_labs/race/
$ scp ./f1-10.launch ubuntu@192.168.40.1:~/
```

## Contributors

- Ryan McCampbell
- Tong Qiu
- Jibang Wu
