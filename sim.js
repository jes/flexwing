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
        shape: new Box(1000, 10, Vec2(-500, -50)),
    });

    wing = world.createBody({
        type: 'dynamic',
        position: Vec2(-50.0, -36.0),
        bullet: true,
    });
    wing.createFixture({
        shape: new Box(3,0.1),
        density: 0.1,
        friction: 0.9,
    });

    trike = world.createBody({
        type: 'dynamic',
        position: Vec2(-49.0, -38.0),
        bullet: true,
    });
    trike.createFixture({
        shape: new Polygon([Vec2(1.5,-1), Vec2(-2.5,-1), Vec2(-1.7,0.6)]),
        density: 0.1,
        friction: 0.1,
    });
    let pivot = world.createJoint(new RevoluteJoint({
        collideConnected: true,
        maxMotorTorque: 0.25,
        motorSpeed: 0,
        enableMotor: true,
    }, wing, trike, wing.getWorldCenter()));
}

let ts = 0;

function step(dt) {
    if (!world)
        return;

    ts += dt;

    airspeed = trike.getLinearVelocity().x;
    vsi = trike.getLinearVelocity().y;
    altitude = trike.getWorldCenter().y;

    // bar pressure
    let p = parseFloat(val('barpressure'));
    let barPoint = wing.getWorldCenter().clone().add(Vec2(1,-1));
    trike.applyForce(Vec2(p,0), barPoint);
    wing.applyForce(Vec2(-p,0), barPoint);
    val('barpressure', p*0.9);

    // air pressure levels out the wing
    wing.applyTorque(-wing.getAngle()/10);
    wing.applyTorque(-wing.getAngularVelocity()/10);

    // thrust
    let power = parseFloat(val('throttle'))*5;
    let thrustVector = Vec2(Math.cos(trike.getAngle()), Math.sin(trike.getAngle()));
    trike.applyForce(Vec2(power*thrustVector.x, power*thrustVector.y), trike.getWorldCenter());

    // TODO: lift and drag on the wing need to depend on the angle of attack relative to the airflow (difference between wing.getAngle() and wing.getLinearVelocity().angle()? )

    // lift
    wing.applyForce(Vec2(0,airspeed*0.75), wing.getWorldCenter());

    // drag on the wing
    wing.applyForce(Vec2(0,-airspeed*airspeed*0.01), wing.getWorldCenter());

    // drag on the trike
    let trikeDrag = trike.getLinearVelocity().clone();
    trikeDrag.normalize();
    trikeDrag.mul(-airspeed*airspeed*0.01);
    trike.applyForce(trikeDrag, trike.getWorldCenter());

    world.step(dt);

    txt('airspeed', sigfigs(airspeed, 2));
    txt('vsi', sigfigs(vsi, 2));
    txt('altitude', sigfigs(altitude, 2));
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
