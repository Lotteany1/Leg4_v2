function file = mixmpc_normalized_urdf(robot,folder)
% 导出仅用于独立 rigidBodyTree 对照的规范树。质量/惯量不变，八个
% 销轴标记按 CAD 实际附着固连到运动连杆；原始 001 文件保持不变。
doc=xmlread(robot.urdf);
nodes=doc.getElementsByTagName('joint');
for k=0:nodes.getLength()-1
    node=nodes.item(k); name=char(node.getAttribute('name'));
    index=find(strcmp({robot.joints.name},name));
    joint=robot.joints(index);
    if strcmp(joint.type,'fixed') && ~strcmp(char(node.getAttribute('type')),'fixed')
        node.setAttribute('type','fixed');
        node.getElementsByTagName('parent').item(0).setAttribute('link',robot.links(joint.parent).name);
        point=node.getElementsByTagName('origin').item(0);
        point.setAttribute('xyz',sprintf('%.17g %.17g %.17g',joint.translation));
        angle=rotm2eul(joint.rotation,'ZYX');
        point.setAttribute('rpy',sprintf('%.17g %.17g %.17g',angle([3 2 1])));
    end
end
file=fullfile(folder,'normalized_001.urdf');
xmlwrite(file,doc);
end
