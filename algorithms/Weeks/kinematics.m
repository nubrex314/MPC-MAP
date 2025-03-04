function [weel] = kinematics(v,u)
%UNTITLED Summary of this function goes here
weelL=(2*v+u)/2;
weelR=(2*v-u)/2;
weel=[weelL,weelR];
end