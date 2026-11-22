function robot = mixmpc_phase_average_wheels(robot)
% 14 态不观测轮相位：保留轮轴自转惯量，将径向质心和横向惯量作相位平均。
% 先将惯量平移至轴心，投影后再移回轴向质心，不能直接平均质心惯量。
for side=1:2
    id=robot.side(side).frames(11);
    link=robot.links(id); joint=robot.joints([robot.joints.child]==id);
    axis=joint.axis; c=link.com;
    I=link.inertia+link.mass*(dot(c,c)*eye(3)-c*c');
    along=axis*axis'; across=eye(3)-along;
    spin=axis'*I*axis; transverse=trace(across*I)/2;
    c=along*c;
    robot.links(id).com=c;
    robot.links(id).inertia=transverse*across+spin*along-link.mass*(dot(c,c)*eye(3)-c*c');
end
end
