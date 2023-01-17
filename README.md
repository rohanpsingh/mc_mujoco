# mujoco interface for mc-rtc
![Screenshot from 2022-08-12 17-09-16](https://user-images.githubusercontent.com/16384313/188832982-1a1263e4-ce33-4dc7-804a-589d151670b3.png)


## Usage

Assuming that you have mujoco installed under `${HOME}/.mujoco/mujoco212`,

```sh
$ git clone --recursive git@github.com:rohanpsingh/mc_mujoco.git
$ cd mc_mujoco
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=$HOME/.mujoco/mujoco212
$ make
$ make install
```
Add the following line to your `~/.bashrc`:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/.mujoco/mujoco212/lib:${HOME}/.mujoco/mujoco212/bin
```
Then, to run the interface:
```sh
$ mc_mujoco
```
---

#### To load additional objects in the scene
There are several steps needed to be followed:
- First, create your object's XML file under `mc_mujoco/robots`. For example, [longtable.xml](robots/longtable.xml)
- Then, create a simple config file with the `xmlModelPath` attribute. For example, [longtable.in.yaml](robots/longtable.in.yaml)
- Then, install your object by adding it to the end of `mc_mujoco/robots/CMakeLists.txt`. For example, add [setup_env_object(box object)](robots/CMakeLists.txt#L15)  

Your object is now created and installed. Next you want to tell `mc-mujoco` to load it and place it at the desired pose.  
Find `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml` (create it manually if needed) and paste the following:
```yaml
objects:
  box1:
    module: "box"
    init_pos:
      translation: [3.1, 0, 0.9]
      rotation: [0, 0, 0]
  box2:
    module: "box"
    init_pos:
      translation: [3.7, 0.2, 0.9]
      rotation: [0, 0, 0]
```
---

#### GUI: Mouse Interaction

An object is selected by left-double-click. The user can then apply forces and torques on the selected object by holding `Ctrl` key and dragging the left-mouse-button for torques and right-mouse-button for forces.

## Example

A basic example of what you can do using this package is [here](https://github.com/rohanpsingh/grasp-fsm-sample-controller).

## Citation
```
@article{singh2022mcmujoco,
  title={mc-mujoco: Simulating Articulated Robots with FSM Controllers in MuJoCo},
  author={Singh, Rohan Pratap and Gergondet, Pierre and Kanehiro, Fumio},
  journal={arXiv preprint arXiv:2209.00274},
  year={2022}
}
```

### Credits

This package includes code from:
- [imgui v1.84.2](https://github.com/ocornut/imgui/)
- [implot v0.11](https://github.com/epezent/implot)
- [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo)
- [pugixml v1.11.4](https://github.com/zeux/pugixml)
- [Roboto font](https://github.com/googlefonts/roboto)
