
use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 0.75;
    let ground_height = 0.1;

    // let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height]);
    // let floor_handle = bodies.insert(rigid_body);
    // let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    // colliders.insert_with_parent(collider, floor_handle, &mut bodies);
    //
    // let rigid_body =
    //     RigidBodyBuilder::fixed().translation(vector![-ground_size - ground_height, ground_size]);
    // let floor_handle = bodies.insert(rigid_body);
    // let collider = ColliderBuilder::cuboid(ground_height, ground_size);
    // colliders.insert_with_parent(collider, floor_handle, &mut bodies);
    //
    // let rigid_body =
    //     RigidBodyBuilder::fixed().translation(vector![ground_size + ground_height, ground_size]);
    // let floor_handle = bodies.insert(rigid_body);
    // let collider = ColliderBuilder::cuboid(ground_height, ground_size);
    // colliders.insert_with_parent(collider, floor_handle, &mut bodies);


    /*
     * Character we will control manually.
     */
    let rigid_body1 = RigidBodyBuilder::new(RigidBodyType::Fixed).translation(vector![0.0, 0.0]);
    let character_handle = bodies.insert(rigid_body1);
    let collider1 = ColliderBuilder::cuboid(0.1, 0.1);
    colliders.insert_with_parent(collider1, character_handle, &mut bodies);

    /*
     * Tethered Ball
     */
    let rad = 0.04;



    let rigid_body2 = RigidBodyBuilder::dynamic().translation(vector![1.0, 0.0]);
    let child_handle = bodies.insert(rigid_body2);
    let collider2 = ColliderBuilder::ball(rad).density(20.0);
    colliders.insert_with_parent(collider2, child_handle, &mut bodies);
/*
    let mut chax = RigidBody::position(RigidBodySet::get(&bodies, character_handle).unwrap()).translation.x;
    let mut chay = RigidBody::position(RigidBodySet::get(&bodies, character_handle).unwrap()).translation.x;
    let mut chix = RigidBody::position(RigidBodySet::get(&bodies, child_handle).unwrap()).translation.x;
    let mut chiy = RigidBody::position(RigidBodySet::get(&bodies, child_handle).unwrap()).translation.y;*/
    // let mut chay = RigidBody::center_of_mass(&RigidBodySet::get(&bodies, character_handle).unwrap()).y;
    // let mut chax = RigidBody::center_of_mass(&RigidBodySet::get(&bodies, character_handle).unwrap()).x;
    // // let chay = RigidBody::position(&RigidBodySet::get(&bodies, character_handle).unwrap()).translation.y;
    // // let chax = RigidBody::position(&RigidBodySet::get(&bodies, character_handle).unwrap()).translation.x;
    // // let chix = RigidBody::position(&RigidBodySet::get(&bodies, child_handle).unwrap()).translation.x;
    // // let chiy = RigidBody::position(&RigidBodySet::get(&bodies, child_handle).unwrap()).translation.y;
    // let mut chix = RigidBody::center_of_mass(&RigidBodySet::get(&bodies, child_handle).unwrap()).x;
    // let mut chiy = RigidBody::center_of_mass(&RigidBodySet::get(&bodies, child_handle).unwrap()).y;



    fn radiussolver(x: f32, y: f32, h: f32, k: f32) -> f32 {
        return ((x - h).powf(2.0) + (y - k).powf(2.0)).sqrt()
    }

    testbed.add_callback(move |_, physics, _, _| {
        // physics.bodies.propagate_modified_body_positions_to_colliders(&mut physics.colliders);
        // chax = RigidBody::position(RigidBodySet::get(&physics.bodies, character_handle).unwrap()).translation.x;
        // chay = RigidBody::position(RigidBodySet::get(&physics.bodies, character_handle).unwrap()).translation.x;
        // chix = RigidBody::position(RigidBodySet::get(&physics.bodies, child_handle).unwrap()).translation.x;
        // chiy = RigidBody::position(RigidBodySet::get(&physics.bodies, child_handle).unwrap()).translation.y;

    //     chay = RigidBody::center_of_mass(&RigidBodySet::get(&physics.bodies, character_handle).unwrap()).y;
    //     chax = RigidBody::center_of_mass(&RigidBodySet::get(&physics.bodies, character_handle).unwrap()).x;
    // // //let chay = RigidBody::position(&RigidBodySet::get(&bodies, character_handle).unwrap()).translation.y;
    // // //let chax = RigidBody::position(&RigidBodySet::get(&bodies, character_handle).unwrap()).translation.x;
    // // // let chix = RigidBody::position(&RigidBodySet::get(&bodies, child_handle).unwrap()).translation.x;
    // // // let chiy = RigidBody::position(&RigidBodySet::get(&bodies, child_handle).unwrap()).translation.y;
    //     chix = RigidBody::center_of_mass(&RigidBodySet::get(&physics.bodies, child_handle).unwrap()).x;
    //     chiy = RigidBody::center_of_mass(&RigidBodySet::get(&physics.bodies, child_handle).unwrap()).y;

        // joint = SpringJointBuilder::motor_position(joint,vector![(chix.powf(2.0) + chax.powf(2.0)).sqrt(), (chiy.powf(2.0) + chay.powf(2.0)).sqrt()], 0.01, 0.01);
            // (
            //     ((radiussolver(chix, chiy, chax, chay)).powf(2.0) - (chiy).powf(2.0) + 2.0*chay*chiy-(chay).powf(2.0)
            // ).sqrt() - chax),
            //                         ((
            //     (radiussolver(chix,chiy,chax,chay)).powf(2.0) - (chix).powf(2.0) + 2.0*chax*chix-(chax).powf(2.0)
            // ).sqrt() - chay)
            // ((((chix - chax).powf(2.0) + (chiy - chay).powf(2.0)).powf(2.0) - (chiy).powf(2.0) + 2.0*chay*chiy-(chay).powf(2.0)) - chax),
            // -((((chix - chax).powf(2.0) + (chiy - chay).powf(2.0)).powf(2.0) - (chix).powf(2.0) + 2.0*chax*chix-(chax).powf(2.0)) - chay)
    let joint = SpringJointBuilder::new()
        //.local_anchor1(point![0.0, 0.0])
        //.local_anchor2(point![0.0, 0.0])
        //.local_frame2(*RigidBody::position(RigidBodySet::get(&physics.bodies, child_handle).unwrap()))
        .local_frame1(*RigidBody::position(RigidBodySet::get(&physics.bodies, character_handle).unwrap()))
        .local_axis2(-Vector::y_axis())
        //.local_axis2(Vector::x_axis())
        .limits([-90.0, 90.0])
        //.motor_position(((rigid_body2.position.translation.x - rigid_body1.position.translation.x).powf(2.0) + (rigid_body2.position.translation.y - rigid_body1.position.translation.y).powf(2.0)).sqrt(), 0.01, 0.012)
        //.motor_position(vector![((rigid_body2.position.translation.x).powf(2.0) - (rigid_body1.position.translation.x).powf(2.0)).sqrt(), ((rigid_body2.position.translation.y).powf(2.0) - (rigid_body1.position.translation.y).powf(2.0)).sqrt()], 0.01, 0.012)
        //.motor_position(vector![0.0,2.0], 0.01, 0.012)
        //.motor_position(vector![(((RigidBody::position(RigidBodySet::get(&bodies, character_handle).unwrap()).translation.x).powf(2.0) - RigidBody::position(RigidBodySet::get(&bodies, child_handle).unwrap()).translation.x).powf(2.0)).sqrt(),(((RigidBody::position(RigidBodySet::get(&bodies, character_handle).unwrap()).translation.y).powf(2.0) - RigidBody::position(RigidBodySet::get(&bodies, child_handle).unwrap()).translation.y).powf(2.0)).sqrt()], 0.01, 0.012)
        //.motor_position(vector![(chax.powf(2.0) + chix.powf(2.0)).sqrt(), (chay.powf(2.0) + chiy.powf(2.0)).sqrt()], 0.01, 0.01)
        //.motor_position(vector![(chax-chix).abs(), (chay-chiy).abs()], 0.0, 0.0)
        //.motor_max_force(((RigidBody::position(RigidBodySet::get(&bodies, character_handle).unwrap()).translation.x - RigidBody::position(RigidBodySet::get(&bodies, child_handle).unwrap()).translation.x).powf(2.0) + (RigidBody::position(RigidBodySet::get(&bodies, character_handle).unwrap()).translation.y - RigidBody::position(RigidBodySet::get(&bodies, child_handle).unwrap()).translation.y).powf(2.0)).sqrt())
        //.motor_max_force(((chax - chix).powi(2) + (chay - chiy).powi(2)).sqrt())
        .motor_position(vector![1.0, 0.0], 0.0, 0.01)

        //joint = SpringJointBuilder::motor_max_force(joint,((chax+chix).powf(2.0)+(chay+chiy).powf(2.0)).sqrt());

        .motor_model(MotorModel::AccelerationBased);
    physics.multibody_joints.insert(character_handle, child_handle, joint, true);
    });

    /*
     * Set up the testbed. ((chix+chax).powf(2.0)+(chiy+chay).powf(2.0))
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    //testbed.set_character_body(character_handle);
    testbed.look_at(point![0.0, 1.0], 100.0);
}
