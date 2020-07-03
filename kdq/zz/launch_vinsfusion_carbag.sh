{
ps_out=`ps -ef | grep  roscore | grep -v 'grep'`
if [ ! -n "$ps_out" ];
then
roscore
else
echo "roscore already run!"
fi
}&
{
 gnome-terminal  --tab -e 'bash -c "cd ~/Workspace/SlamCodes/OpenSource/CatkinWs_VinsMono;source devel/setup.bash;roslaunch vins_estimator zerozero.launch;exec bash"'
}&
sleep 2s
{
if [ "$1" = "rviz" ]
then
echo "run rviz"
  gnome-terminal --tab -e 'bash -c "cd ~/Workspace/SlamCodes/OpenSource/CatkinWs_VinsMono;source devel/setup.bash;roslaunch vins_estimator vins_rviz.launch;"'
else
echo "no rviz"
fi
}&
sleep 2s
{ 
if [ "$2" = "loop" ]
then
echo "loop"
 gnome-terminal -x zsh -c "cd ~/Workspace/SlamCodes/OpenSource/CatkinWs_VinsFusion;source devel/setup.zsh;rosrun loop_fusion loop_fusion_node  ~/Workspace/SlamCodes/OpenSource/CatkinWs_VinsFusion/src/VINS-Fusion/config/vi_car/vi_car.yaml"
else
echo "no loop!"
fi
}&
{
  gnome-terminal -x zsh -c "cd /home/kdq/Workspace/SlamCodes/OpenSource/CatkinWs_VinsMono;source devel/setup.zsh;rosrun zz_replay zz_replay ./src/VINS-Mono/config/zz/zerozero_config.yaml"
}
