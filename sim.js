const { World, Vec2, Box, Circle, Polygon, DistanceJoint, RevoluteJoint, Settings } = planck;

let world;
let renderer;
let ground;
let wing;
let trike;

function setupSimulation(v) {
    world = new World({
        gravity: Vec2(0.0, -9.81),
        allowSleep: false,
    });
    renderer = new Renderer(world, 'canvas');

    ground = world.createBody({
        type: 'static',
        position: Vec2(0.0, 0.0),
    });
    ground.createFixture({
        shape: new Box(10000, 10, Vec2(-5000, -5)),
    });

    wing = world.createBody({
        type: 'dynamic',
        position: Vec2(0, 9),
        bullet: true,
    });
    wing.createFixture({
        shape: new Box(3,0.1),
        density: 60,
        friction: 0.9,
    });

    trike = world.createBody({
        type: 'dynamic',
        position: Vec2(1, 7),
        bullet: true,
    });
    trike.createFixture({
        shape: new Polygon([Vec2(1.5,-1), Vec2(-2.5,-1), Vec2(-1.7,0.6)]),
        density: 100,
        friction: 0.1,
    });
    world.createJoint(new RevoluteJoint({
        collideConnected: true,
        maxMotorTorque: 0.25, // friction in the joint?
        motorSpeed: 0,
        enableMotor: false,
    }, wing, trike, wing.getWorldCenter()));
}

let ts = 0;

let sliding = false;
elem('barpressure').addEventListener('mousedown', function() {
    sliding = true;
});
elem('barpressure').addEventListener('mouseup', function() {
    sliding = false;
});

function step(dt) {
    if (!world)
        return;

    ts += dt;

    airspeed = wing.getLinearVelocity().length();
    vsi = wing.getLinearVelocity().y;
    altitude = wing.getWorldCenter().y;

    // bar pressure
    let p = parseFloat(val('barpressure'))*-100;
    let barPoint = wing.getWorldCenter().clone().add(Vec2(1,-1));
    trike.applyForce(Vec2(p,0), barPoint);
    wing.applyForce(Vec2(-p,0), barPoint);
    if (!sliding)
        val('barpressure', val('barpressure')*0.9);

    // thrust
    let power = parseFloat(val('throttle'))*5000;
    let thrustVector = Vec2(Math.cos(trike.getAngle()), Math.sin(trike.getAngle()));
    thrustVector.mul(power);
    trike.applyForce(thrustVector, trike.getWorldCenter());
    txt('thrust', sigfigs(thrustVector.x,3) + "," + sigfigs(thrustVector.y,3));

    // drag on the trike
    let trikeDrag = trike.getLinearVelocity().clone();
    trikeDrag.normalize();
    trikeDrag.mul(-airspeed*airspeed*10);
    trike.applyForce(trikeDrag, trike.getWorldCenter());

    // angle of attack is difference between wing angle and movement angle
    let w = wing.getLinearVelocity();
    let aoa = wing.getAngle() - Math.atan2(w.y, w.x);

    // the total force on the wing depends on the airspeed and angle of attack
    let wingForce = liftForce(airspeed, aoa);

    // and the direction of that force is always 90 degrees to the angle of attack
    let wingForceAngle;
    if (aoa > 0) wingForceAngle = aoa + Math.PI/2;
    else wingForceAngle = aoa - Math.PI/2;
    wingForceAngle += Math.atan2(w.y, w.x);

    // lift & drag
    let liftVector = Vec2(Math.cos(wingForceAngle), Math.sin(wingForceAngle));
    liftVector.mul(wingForce);
    txt('lift', sigfigs(liftVector.x,3) + "," + sigfigs(liftVector.y,3));
    wing.applyForce(liftVector, wing.getWorldCenter());

    // air pressure levels out the wing
    wing.applyTorque(-aoa/10*airspeed/10);
    wing.applyTorque(-wing.getAngularVelocity()*10);

    world.step(dt);

    txt('airspeed', sigfigs(airspeed*2.236, 2)); // 1 m/s = 2.236 mph
        txt('vsi', sigfigs(vsi*3.281, 2)); // 1 m = 3.281 feet
    txt('altitude', sigfigs(altitude*3.281, 2)); // 1 m = 3.281 feet
    txt('aoa', sigfigs(aoa*180/Math.PI, 2));
}

function liftForce(airspeed, aoa) {
    let C_l = aoa; // coefficient of lift
    if (C_l > 1) C_l = 1;
    else if (C_l < -1) C_l = -1;

    return C_l * airspeed * airspeed * 50;
}

window.setInterval(function() {
    if (!world)
        return;
    let iters = 5;
    for (let i = 0; i < iters; i++) {
        step(1/(60*iters));
    }
    for (let scope of scopes) {
        scope.update(1/60);
        scope.draw();
    }
}, 1000/60);
