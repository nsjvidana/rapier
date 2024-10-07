use std::{f32::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, PI}, num::NonZeroUsize};

use nalgebra::Vector3;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use wasm_bindgen::UnwrapThrowExt;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();


    //ground
    let ground_size = 10.;
    let ground_height = 1.;
    let ground = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height/2., 0.0]);
    let ground_body = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
        colliders.insert_with_parent(collider, ground_body, &mut bodies);


    let radius = 0.1;
    let head_radius = 0.3;
    let head_total_height = 1.0;
    let head_half_height = (head_total_height-head_radius*2.)/2.;
    let arm_upper_len = 1.;
    let arm_lower_len = 1.25;
    let spine_len = 4.;
    let num_spine_segments = 2;
    let spine_seg_len = spine_len/(1. + num_spine_segments as f32);
    let leg_len = 3.05;
    let leg_upper_len = leg_len * 0.49;
    let leg_lower_len = leg_len*0.51;
    let motor_vel = 10f32.to_radians();
    let spine_seg_coll = ColliderBuilder::capsule_y((spine_seg_len/2.)-radius, radius);
    let body_spawn_loc = vector![0., 6., 0.];

    let seg_coll_handle = colliders.insert(
        spine_seg_coll.clone()
    );

    let target_vel = 90f32.to_radians();
    let stiffness = 1.;
    let damping = 0.;
    let spine_root;
    let spine_end;

    let mut mbj_handle_opt = None;

    {//spine
        let fixed = bodies.insert(RigidBodyBuilder::fixed().translation(body_spawn_loc));
        spine_root = bodies.insert(RigidBodyBuilder::dynamic());
            colliders.set_parent(seg_coll_handle, Some(spine_root), &mut bodies);
        multibody_joints.insert(fixed, spine_root, FixedJoint::new(), true);
        let joint = SphericalJointBuilder::new()
            .local_anchor1(point![0., spine_seg_len/2., 0.])
            .local_anchor2(point![0., -spine_seg_len/2., 0.])
            .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
            .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
            .motor(JointAxis::AngZ, 0., target_vel, stiffness, damping)
            .build();
        let mut prev = spine_root;
        for _ in 0..num_spine_segments {
            let seg_coll_handle = colliders.insert(
                spine_seg_coll.clone()
            );
            let new_seg = bodies.insert(RigidBodyBuilder::dynamic().translation(body_spawn_loc));
            colliders.set_parent(seg_coll_handle, Some(new_seg), &mut bodies);
            mbj_handle_opt = multibody_joints.insert(prev, new_seg, joint, true);
            prev = new_seg;
        }
       spine_end = prev;
    }


    let joint_collider = ColliderBuilder::ball(radius);

    {//arms
        let arm_r_upper;

        let shldx_j = RevoluteJointBuilder::new(Vector::x_axis())
            .local_anchor1(point![0., spine_seg_len/2., 0.])
            .limits([-150f32.to_radians(), 60f32.to_radians()])
            .motor(0., target_vel, stiffness, damping);
        let shldy_j = RevoluteJointBuilder::new(Vector::y_axis())
            .limits([-90f32.to_radians(), 90f32.to_radians()])
            .motor(0., target_vel, stiffness, damping);

        let upper_arm_coll = ColliderBuilder::capsule_x((arm_upper_len/2.)-radius, radius);
        let lower_arm_coll = ColliderBuilder::capsule_x((arm_lower_len/2.)-radius, radius);

        {//right arm
            let mut rb_builder = RigidBodyBuilder::dynamic().translation(vector![1., leg_len+spine_len, 1.]);
            {//upper arm
                let r_shld_j = SphericalJointBuilder::new()
                    .local_anchor1(point![0., spine_seg_len/2., 0.])
                    .local_anchor2(point![-arm_upper_len/2., 0., 0.])
                    .limits(JointAxis::AngX, [-FRAC_PI_2-FRAC_PI_4, FRAC_PI_2])
                    .limits(JointAxis::AngY, [-FRAC_PI_2, FRAC_PI_2+FRAC_PI_4])
                    .limits(JointAxis::AngZ, [-FRAC_PI_2, FRAC_PI_2])
                    .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
                    .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
                    .motor(JointAxis::AngZ, 0., target_vel, stiffness, damping);

                let shldz_j = RevoluteJointBuilder::new(Vector::z_axis())
                    .local_anchor2(point![-arm_upper_len/2., 0., 0.])
                    .limits([-90f32.to_radians(), 90f32.to_radians()])
                    .motor(0., target_vel, stiffness, damping);

                arm_r_upper = bodies.insert(rb_builder.clone());
                colliders.insert_with_parent(
                    upper_arm_coll.clone(),
                    arm_r_upper,
                    &mut bodies
                );
            
                multibody_joints.insert(spine_end, arm_r_upper, r_shld_j, true);
            }


            {//lower arm
                rb_builder = rb_builder.translation(vector![2., leg_len+spine_len, 1.]);
                let elbx_j = RevoluteJointBuilder::new(Vector::x_axis())
                    .local_anchor1(point![arm_upper_len/2., 0., 0.])
                    .limits([0., 45f32.to_radians()])
                    .motor(0., target_vel, stiffness, damping);
                let elby_j = RevoluteJointBuilder::new(Vector::y_axis())
                    .local_anchor2(point![-arm_lower_len/2., 0., 0.])
                    .limits([0., 180f32.to_radians()])
                    .motor(0., target_vel, stiffness, damping);
            
                let elbx = bodies.insert(rb_builder.clone());
                    colliders.insert_with_parent(joint_collider.clone(), elbx, &mut bodies);
                let arm_r_lower = bodies.insert(rb_builder.clone());
                    colliders.insert_with_parent(
                        lower_arm_coll.clone(),
                        arm_r_lower,
                        &mut bodies
                    );

                multibody_joints.insert(arm_r_upper, elbx, elbx_j, true);
                multibody_joints.insert(elbx, arm_r_lower, elby_j, true);
            }
        }


        {//left arm
            let arm_l_upper;
            let rb_builder = RigidBodyBuilder::dynamic().translation(vector![-1., leg_len+spine_len, -1.]);
            {//upper
                let shldz_j = RevoluteJointBuilder::new(Vector::z_axis())
                    .local_anchor2(point![arm_upper_len/2., 0., 0.])
                    .limits([-90f32.to_radians(), 90f32.to_radians()])
                    .motor(0., target_vel, stiffness, damping);

                let l_shld_j = SphericalJointBuilder::new()
                    .local_anchor1(point![0., spine_seg_len/2., 0.])
                    .local_anchor2(point![arm_upper_len/2., 0., 0.])
                    .limits(JointAxis::AngX, [-FRAC_PI_2-FRAC_PI_4, FRAC_PI_2])
                    .limits(JointAxis::AngY, [-FRAC_PI_2-FRAC_PI_4, FRAC_PI_2])
                    .limits(JointAxis::AngZ, [-FRAC_PI_2, FRAC_PI_2])
                    .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
                    .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
                    .motor(JointAxis::AngZ, 0., target_vel, stiffness, damping);

                arm_l_upper = bodies.insert(rb_builder.clone());
                    colliders.insert_with_parent(upper_arm_coll, arm_l_upper, &mut bodies);

                
                multibody_joints.insert(spine_end, arm_l_upper, l_shld_j, true);
            }


            {//lower
                let elby_j = RevoluteJointBuilder::new(Vector::y_axis())
                    .local_anchor1(point![-arm_upper_len/2., 0., 0.])
                    .limits([-FRAC_PI_2+FRAC_PI_4, 0.])
                    .motor(0., target_vel, stiffness, damping);
                let elbx_j = RevoluteJointBuilder::new(Vector::x_axis())
                    .local_anchor2(point![arm_lower_len/2., 0., 0.])
                    .limits([-FRAC_PI_2, FRAC_PI_2])
                    .motor(0., target_vel, stiffness, damping);


                let elbx = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![0., 0., 1.]));
                    colliders.insert_with_parent(joint_collider.clone(), elbx, &mut bodies);
                let arm_l_lower = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![0., 0., 1.]));
                    colliders.insert_with_parent(
                        lower_arm_coll.clone(),
                        arm_l_lower,
                        &mut bodies
                    );

                
                multibody_joints.insert(arm_l_upper, elbx, elby_j, true);
                multibody_joints.insert(elbx, arm_l_lower, elbx_j, true);
            }
        }
    }

    {//legs
        let mut hipx_j = RevoluteJointBuilder::new(Vector3::x_axis())
            .local_anchor1(point![0., -spine_seg_len/2., 0.])
            .limits([-FRAC_PI_6, FRAC_PI_2])
            .motor(0., target_vel, stiffness, damping);
        let mut hipy_j = RevoluteJointBuilder::new(Vector3::y_axis())
            .limits([0., FRAC_PI_2])
            .motor(0., target_vel, stiffness, damping);
        let mut hipz_j = RevoluteJointBuilder::new(Vector3::z_axis())
            .local_anchor2(point![0., leg_upper_len/2., 0.])
            .limits([-FRAC_PI_4, 0.])
            .motor(-FRAC_PI_6/2., target_vel, stiffness, damping);
        let mut kneex_j = RevoluteJointBuilder::new(Vector3::x_axis())
            .local_anchor1(point![0., -leg_upper_len/2., 0.])
            .local_anchor2(point![0., leg_lower_len/2., 0.])
            .limits([-FRAC_PI_2-FRAC_PI_4, 0.])
            .motor(0., target_vel, stiffness, damping);
        let upper_leg_collider = ColliderBuilder::capsule_y((leg_upper_len/2.)-radius, radius);
        let lower_leg_collider = ColliderBuilder::capsule_y((leg_upper_len/2.)-radius, radius);

        {//right leg

            let r_hip = SphericalJointBuilder::new()
                .local_anchor1(point![0., -spine_seg_len/2., 0.])
                .local_anchor2(point![0., leg_upper_len/2., 0.])
                .limits(JointAxis::AngX, [-FRAC_PI_2, FRAC_PI_2])
                .limits(JointAxis::AngY, [-FRAC_PI_2, 0.])
                .limits(JointAxis::AngZ, [-FRAC_PI_4, FRAC_PI_6/2.])
                .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
                .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
                .motor(JointAxis::AngZ, FRAC_PI_6/2., target_vel, stiffness, damping);


            let leg_r_upper = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![0., 5., 1.]));
                colliders.insert_with_parent(upper_leg_collider.clone(), leg_r_upper, &mut bodies);
            let leg_l_lower = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![0., 5., 1.]));
                colliders.insert_with_parent(lower_leg_collider.clone(), leg_l_lower, &mut bodies);

            multibody_joints.insert(spine_root, leg_r_upper, r_hip, true);
            multibody_joints.insert(leg_r_upper, leg_l_lower, kneex_j, true);
        }

        {//left leg

            let l_hip = SphericalJointBuilder::new()
                .local_anchor1(point![0., -spine_seg_len/2., 0.])
                .local_anchor2(point![0., leg_upper_len/2., 0.])
                .limits(JointAxis::AngX, [-FRAC_PI_2, FRAC_PI_2])
                .limits(JointAxis::AngY, [0., FRAC_PI_2])
                .limits(JointAxis::AngZ, [-FRAC_PI_6/2., FRAC_PI_4])
                .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
                .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
                .motor(JointAxis::AngZ, -FRAC_PI_6/2., target_vel, stiffness, damping);

            let leg_l_upper = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![1., 5., 1.]));
                colliders.insert_with_parent(upper_leg_collider, leg_l_upper, &mut bodies);
            
            let leg_l_lower = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![1., 5., 1.]));
                colliders.insert_with_parent(lower_leg_collider, leg_l_lower, &mut bodies);
            
            multibody_joints.insert(spine_root, leg_l_upper, l_hip, true);
            multibody_joints.insert(leg_l_upper, leg_l_lower, kneex_j, true);
        }
    }

    {//head
        // let neck_j = SphericalJointBuilder::new()
        //     .local_anchor1(point![0., spine_seg_len/2., 0.])
        //     .local_anchor2(point![0., -(head_half_height+head_radius)/2., 0.])
        //     .limits(JointAxis::AngX, [-FRAC_PI_4, FRAC_PI_3])
        //     .limits(JointAxis::AngY, [-PI, PI])
        //     .limits(JointAxis::AngZ, [-FRAC_PI_4, FRAC_PI_4])
        //     .motor(JointAxis::AngX, 0., motor_vel, 1., 0.)
        //     .motor(JointAxis::AngY, 0., motor_vel, 1., 0.)
        //     .motor(JointAxis::AngZ, 0., motor_vel, 1., 0.);
        // let head_collider = ColliderBuilder::capsule_y(head_half_height, head_radius);
        // let head = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![0., 1.5, -1.5]));
        //     colliders.insert_with_parent(head_collider, head, &mut bodies);
        // multibody_joints.insert(spine_end, head, neck_j, true);
    }

    let mb = multibody_joints.get_mut(mbj_handle_opt.unwrap()).unwrap().0;
        mb.set_self_contacts_enabled(false);

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    let integration_params = testbed.integration_parameters_mut();
        integration_params.num_solver_iterations = NonZeroUsize::new(12).unwrap();
        integration_params.num_internal_stabilization_iterations = 4;
    testbed.look_at(point![15., 15., 15.], point![0.0, 2., 0.0]);
}
