# [WIP] Mujoco interface for mc-rtc

Add this to your mc-rtc configuration yaml:  

```yaml
MUJOCO:
  xmlModelPath: "<path_to_your_xml_model>"
  pdGainsPath: "<path_to_your_pdgains>"
```

### Usage:  
Assuming that you have mujoco installed under `${HOME}/.mujoco/mujoco200`,

```sh
$ git clone git@github.com:rohanpsingh/mc_mujoco.git
$ cd mc_mujoco
$ mkdir build && cd build
$ cmake ..
$ make
```
Set the environment variabe `MUJOCO_KEY_PATH` to the path where your mjkey.txt exists.

Then, to run the interface:  
```sh
$ cd mc_mujoco/build/src
$ ./mc_mujoco
```

