function contact = mixmpc_contact_model(model, kin, terrain)
% Instantaneous material-point contact of a thin round wheel with a plane.
% terrain.origin/normal are 3x2 plane points and unit normals. terrain.mu
% contains the two friction coefficients. Rows of J are forward/lateral/normal.
assert(isequal(size(terrain.origin),[3 2]) && isequal(size(terrain.normal),[3 2]) && ...
    isequal(size(terrain.mu),[2 1]) && all(isfinite([terrain.origin(:);terrain.normal(:);terrain.mu])) && ...
    all(abs(vecnorm(terrain.normal)-1)<1e-10) && all(terrain.mu>=0), 'Invalid contact planes.');
J = zeros(6,numel(kin.v));
bias = zeros(6,1);
gap = zeros(2,1);
point = zeros(3,2);
basis = zeros(3,3,2);
for side = 1:2
    id = model.wheel_link(side);
    n = terrain.normal(:,side);
    axle = kin.pose.R(:,:,id)*model.wheel_axis(:,side);
    axle_dot = cross(kin.omega(:,id),axle);
    normal_part = dot(axle,n);
    radial = n-normal_part*axle;
    radial_dot = -normal_part*axle_dot-dot(axle_dot,n)*axle;
    rho = norm(radial);
    assert(rho>1e-3,'Thin-wheel contact is undefined for an axle almost normal to the plane.');
    unit = radial/rho;
    r = -model.wheel_radius(side)*unit;
    r_dot = -model.wheel_radius(side)/rho*(radial_dot-unit*dot(unit,radial_dot));

    forward = sign(model.wheel_axis(2,side))*cross(axle,n)/rho;
    lateral = cross(n,forward);
    B = [forward lateral n];
    W = kin.Jw(:,:,id);
    J_world = kin.Jv(:,:,id)+cross(W,repmat(r,1,size(W,2)),1);
    contact_velocity = J_world*kin.v;
    omega = kin.omega(:,id);
    % The radius points to the lowest rim point, so r_dot is the derivative
    % of that geometric radius. A material-point centripetal term here would
    % incorrectly constrain the changing wheel phase.
    b_world = kin.bv(:,id)+cross(kin.bw(:,id),r)+cross(omega,r_dot);
    forward_dot = sign(model.wheel_axis(2,side))*cross(axle_dot,n)/rho ...
                - forward*dot(unit,radial_dot)/rho;
    B_dot = [forward_dot cross(n,forward_dot) zeros(3,1)];
    rows = 3*(side-1)+(1:3);
    J(rows,:) = B'*J_world;
    bias(rows) = B'*b_world+B_dot'*contact_velocity;
    point(:,side) = kin.pose.p(:,id)+r;
    gap(side) = dot(n,point(:,side)-terrain.origin(:,side));
    basis(:,:,side) = B;
end
contact = struct('gap',gap,'point',point,'basis',basis,'J',J,'bias',bias, ...
    'velocity',reshape(J*kin.v,3,2),'mu',terrain.mu);
end
