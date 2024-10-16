use std::{f32::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, PI}, num::NonZeroUsize};

use nalgebra::Vector3;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

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


    //geometry parameters
    let radius = 0.1;
    let spine_len = 2.;
    let arm_len = spine_len*1.333;
    let arm_upper_len = arm_len*0.45;
    let arm_lower_len = arm_len - arm_upper_len;
    let leg_len = spine_len*1.9;
    let num_spine_segments = 2;
    let neck_len = spine_len/6.;
    let head_diameter = (spine_len/2. + neck_len/2.)*0.95;

    let head_radius = head_diameter/2.;
    let spine_seg_len = spine_len/(1. + num_spine_segments as f32);
    let leg_upper_len = leg_len * 0.49;
    let leg_lower_len = leg_len*0.51;
    let motor_vel = 10f32.to_radians();
    let spine_seg_coll = ColliderBuilder::capsule_y((spine_seg_len/2.)-radius, radius);
    let body_spawn_loc = vector![0., 6., 0.];

    let joint_collider = ColliderBuilder::ball(radius);

    let target_vel = 90f32.to_radians();
    let stiffness = 1.;
    let damping = 0.01;
    let spine_end;
    let spine_end_pos;
    

    let mut mbj_handle_opt = None;

    
    let spine_root = bodies.insert(RigidBodyBuilder::dynamic().translation(body_spawn_loc));
        colliders.insert_with_parent(
            spine_seg_coll.clone(),
            spine_root,
            &mut bodies
        );
        colliders.insert_with_parent(
            joint_collider.clone().translation(vector![0., -spine_seg_len/2., 0.]),
            spine_root,
            &mut bodies
        );
    
    
    let leg_l_lower;
    let leg_r_lower;
    {//legs
        let kneex_j = RevoluteJointBuilder::new(Vector3::x_axis())
            .local_anchor1(point![0., -leg_upper_len/2., 0.])
            .local_anchor2(point![0., leg_lower_len/2., 0.])
            .limits([-FRAC_PI_2-FRAC_PI_4, 0.])
            .motor(0., target_vel, stiffness, damping);
        let upper_leg_collider = ColliderBuilder::capsule_y((leg_upper_len/2.)-radius, radius);
        let lower_leg_collider = ColliderBuilder::capsule_y((leg_upper_len/2.)-radius, radius);

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

            let leg_l_upper = bodies.insert(RigidBodyBuilder::dynamic().translation(body_spawn_loc+vector![-radius*2., -leg_upper_len, 0.]));
                colliders.insert_with_parent(upper_leg_collider.clone(), leg_l_upper, &mut bodies);
            
            leg_l_lower = bodies.insert(RigidBodyBuilder::dynamic().translation(body_spawn_loc+vector![-radius*5., -leg_upper_len, 0.]));
                colliders.insert_with_parent(lower_leg_collider.clone(), leg_l_lower, &mut bodies);
                colliders.insert_with_parent(
                    joint_collider.clone().translation(vector![0., leg_lower_len/2., 0.]),
                    leg_l_lower,
                    &mut bodies
                );
            
            multibody_joints.insert(spine_root, leg_l_upper, l_hip, true);
            multibody_joints.insert(leg_l_upper, leg_l_lower, kneex_j, true);
        }

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


            let leg_r_upper = bodies.insert(RigidBodyBuilder::dynamic().translation(body_spawn_loc+vector![radius*2., -leg_upper_len, 0.]));
                colliders.insert_with_parent(upper_leg_collider, leg_r_upper, &mut bodies);
            leg_r_lower = bodies.insert(RigidBodyBuilder::dynamic().translation(body_spawn_loc+vector![radius*5., -leg_upper_len, 0.]));
                colliders.insert_with_parent(lower_leg_collider, leg_r_lower, &mut bodies);
                colliders.insert_with_parent(
                    joint_collider.clone().translation(vector![0., leg_lower_len/2., 0.]),
                    leg_r_lower,
                    &mut bodies
                );

            multibody_joints.insert(spine_root, leg_r_upper, r_hip, true);
            multibody_joints.insert(leg_r_upper, leg_r_lower, kneex_j, true);
        }
    }

    {//spine
        let joint = SphericalJointBuilder::new()
            .local_anchor1(point![0., spine_seg_len/2., 0.])
            .local_anchor2(point![0., -spine_seg_len/2., 0.])
            .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
            .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
            .motor(JointAxis::AngZ, 0., target_vel, stiffness, damping)
            .build();
        let mut prev = spine_root;
        for i in 0..num_spine_segments {
            let new_seg = bodies.insert(RigidBodyBuilder::dynamic()
                .translation(body_spawn_loc + vector![0., (i as f32 + 1.)*(spine_seg_len*1.1), 0.])
            );
            colliders.insert_with_parent(
                spine_seg_coll.clone(),
                new_seg,
                &mut bodies
            );
            colliders.insert_with_parent(
                joint_collider.clone().translation(vector![0., -spine_seg_len/2., 0.]),
                new_seg,
                &mut bodies
            );

            mbj_handle_opt = multibody_joints.insert(prev, new_seg, joint, true);
            prev = new_seg;
        }
       spine_end = prev;
       
       let spine_end_y = (num_spine_segments as f32)*(spine_seg_len*1.1) + spine_seg_len/2.;
       spine_end_pos = body_spawn_loc + vector![0., spine_end_y, 0.];
    }

    {//arms
        let arm_r_upper;

        let upper_arm_coll = ColliderBuilder::capsule_x((arm_upper_len/2.)-radius, radius);
        let lower_arm_coll = ColliderBuilder::capsule_x((arm_lower_len/2.)-radius, radius);

        {//left arm
            let arm_l_upper;
            let rb_builder = RigidBodyBuilder::dynamic().translation(vector![-1., leg_len+spine_len, -1.]);
            
            let upper_arm_pos = spine_end_pos - vector![arm_upper_len/2. + radius, 0., 0.];
            {//upper
                let l_shld_j = SphericalJointBuilder::new()
                    .local_anchor1(point![0., spine_seg_len/2., 0.])
                    .local_anchor2(point![arm_upper_len/2., 0., 0.])
                    .limits(JointAxis::AngX, [-FRAC_PI_2-FRAC_PI_4, FRAC_PI_2])
                    .limits(JointAxis::AngY, [-FRAC_PI_2-FRAC_PI_4, FRAC_PI_2])
                    .limits(JointAxis::AngZ, [-FRAC_PI_2, FRAC_PI_2])
                    .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
                    .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
                    .motor(JointAxis::AngZ, 0., target_vel, stiffness, damping);

                arm_l_upper = bodies.insert(rb_builder.clone().translation(upper_arm_pos));
                    colliders.insert_with_parent(upper_arm_coll.clone(), arm_l_upper, &mut bodies);

                
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


                let elb_pos = upper_arm_pos - vector![arm_upper_len/2. + radius*2., 0., 0.];
                let elbx = bodies.insert(RigidBodyBuilder::dynamic().translation(elb_pos));
                    // colliders.insert_with_parent(joint_collider.clone(), elbx, &mut bodies);
                let arm_l_lower = bodies.insert(RigidBodyBuilder::dynamic()
                    .translation(elb_pos - vector![arm_lower_len/2. + radius*2., 0., 0.])
                );
                colliders.insert_with_parent(
                    lower_arm_coll.clone(),
                    arm_l_lower,
                    &mut bodies
                );
                colliders.insert_with_parent(
                    
                    joint_collider.clone().translation(vector![arm_lower_len/2., 0., 0.]),
                    arm_l_lower,
                    &mut bodies
                );

                
                multibody_joints.insert(arm_l_upper, elbx, elby_j, true);
                multibody_joints.insert(elbx, arm_l_lower, elbx_j, true);
            }
        }
    
        {//right arm
            let mut rb_builder = RigidBodyBuilder::dynamic();

            let upper_arm_pos = spine_end_pos + vector![arm_upper_len/2. + radius, 0., 0.];
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

                arm_r_upper = bodies.insert(rb_builder.clone().translation(upper_arm_pos));
                colliders.insert_with_parent(
                    upper_arm_coll,
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
                
                let elb_pos = upper_arm_pos + vector![arm_upper_len/2. + radius*2., 0., 0.];
                let elbx = bodies.insert(rb_builder.clone()
                    .translation(elb_pos)
                );
                let arm_r_lower = bodies.insert(rb_builder.clone()
                    .translation(elb_pos + vector![arm_lower_len/2. + radius*2., 0., 0.])
                );
                colliders.insert_with_parent(
                    lower_arm_coll,
                    arm_r_lower,
                    &mut bodies
                );
                colliders.insert_with_parent(
                    joint_collider.clone().translation(vector![-arm_lower_len/2., 0., 0.]),
                    arm_r_lower,
                    &mut bodies
                );

                multibody_joints.insert(arm_r_upper, elbx, elbx_j, true);
                multibody_joints.insert(elbx, arm_r_lower, elby_j, true);
            }
        }

    }

    {//head
        let neck_pos = spine_end_pos + vector![0., neck_len/2. + radius, 0.];
        let neck_collider = ColliderBuilder::capsule_y((neck_len/2.)-radius, radius);
        let neck = bodies.insert(RigidBodyBuilder::dynamic().translation(neck_pos));
        colliders.insert_with_parent(
            neck_collider,
            neck,
            &mut bodies
        );
        
        let head_pos = neck_pos + vector![0., neck_len/2. + head_radius + radius, 0.];
        let head_collider = ColliderBuilder::ball(head_radius);
        let head = bodies.insert(RigidBodyBuilder::dynamic()
            .translation(head_pos)
        );
            colliders.insert_with_parent(head_collider, head, &mut bodies);
            
        let spine_to_neck_j = SphericalJointBuilder::new()
            .local_anchor1(point![0., spine_seg_len/2., 0.])
            .local_anchor2(point![0., -neck_len/2., 0.])
            .motor(JointAxis::AngX, 0., target_vel, stiffness, damping)
            .motor(JointAxis::AngY, 0., target_vel, stiffness, damping)
            .motor(JointAxis::AngZ, 0., target_vel, stiffness, damping);
        let neck_j = SphericalJointBuilder::new()
            .local_anchor1(point![0., neck_len/2., 0.])
            .local_anchor2(point![0., -head_radius, 0.])
            .limits(JointAxis::AngX, [-FRAC_PI_4, FRAC_PI_3])
            .limits(JointAxis::AngY, [-PI, PI])
            .limits(JointAxis::AngZ, [-FRAC_PI_4, FRAC_PI_4])
            .motor(JointAxis::AngX, 0., motor_vel, 1., 0.)
            .motor(JointAxis::AngY, 0., motor_vel, 1., 0.)
            .motor(JointAxis::AngZ, 0., motor_vel, 1., 0.);

        multibody_joints.insert(spine_end, neck, spine_to_neck_j, true);
        multibody_joints.insert(neck, head, neck_j, true);
    }

    let mb = multibody_joints.get_mut(mbj_handle_opt.unwrap()).unwrap().0;
        mb.set_self_contacts_enabled(false);

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    let integration_params = testbed.integration_parameters_mut();
        integration_params.num_solver_iterations = NonZeroUsize::new(6).unwrap();
    testbed.look_at(point![0., body_spawn_loc.y, 15.], point![0.0, body_spawn_loc.y, 0.0]);
}
