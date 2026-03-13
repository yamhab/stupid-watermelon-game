use macroquad::prelude::*;
use rapier2d::prelude::*;

const FRUIT_COLORS: [Color; 11] = [
    DARKBLUE, BLACK, PURPLE, BROWN, DARKPURPLE, ORANGE, RED, PINK, YELLOW, GREEN, DARKGREEN,
];

#[derive(Clone, Copy, Debug, PartialEq)]
struct Fruit {
    kind: usize,
    radius: f32,
    color: Color,
    points: u32,
}

impl Fruit {
    fn new(kind: usize) -> Self {
        Self {
            kind,
            radius: kind as f32 / 50.0 + 0.025,
            color: FRUIT_COLORS[kind - 1],
            points: kind as u32 * 2,
        }
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Stupid Watermelon Game".to_string(),
        window_width: 700,
        window_height: 600,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut physics_pipeline = PhysicsPipeline::new();
    let gravity = Vector::new(0.0, 5.0);
    let integration_parameters = IntegrationParameters::default();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    create_container(&mut rigid_body_set, &mut collider_set);
    let mut current_fruit_handle = Some(create_fruit(
        &mut rigid_body_set,
        &mut collider_set,
        0.0,
        Fruit::new(rand::gen_range(1, 5)),
    ));
    let mut next_fruit = Fruit::new(rand::gen_range(1, 5));
    let mut score = 0;
    let mut drop_time = 0.0;
    let mut points = 0;
    let mut points_time = -2.0;

    loop {
        physics_pipeline.step(
            gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        drop_current_fruit(
            &mut rigid_body_set,
            &mut current_fruit_handle,
            &mut drop_time,
        );
        respawn_current_fruit(
            &mut rigid_body_set,
            &mut collider_set,
            &mut current_fruit_handle,
            &mut next_fruit,
            drop_time,
        );
        move_current_fruit(&mut rigid_body_set, current_fruit_handle);
        collide_fruit(
            &mut island_manager,
            &narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            current_fruit_handle,
            &mut score,
            &mut points,
            &mut points_time,
        );
        if check_game_over(&rigid_body_set) {
            return;
        }

        clear_background(LIGHTGRAY);
        draw_fruits(&rigid_body_set);
        draw_container();
        draw_next_fruit(next_fruit);
        draw_score(&mut score, points, &mut points_time);

        next_frame().await;
    }
}

fn create_container(rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) {
    let container_collider = ColliderBuilder::polyline(
        vec![
            Vector::new(0.75, 0.5),
            Vector::new(1.0, 2.0),
            Vector::new(2.0, 2.0),
            Vector::new(2.25, 0.5),
        ],
        None,
    )
    .build();
    let container_rigid_body = RigidBodyBuilder::fixed().build();
    let container_rigid_body_handle = rigid_body_set.insert(container_rigid_body);
    collider_set.insert_with_parent(
        container_collider,
        container_rigid_body_handle,
        rigid_body_set,
    );
}

fn create_fruit(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    gravity_scale: f32,
    fruit: Fruit,
) -> RigidBodyHandle {
    let fruit_collider = ColliderBuilder::ball(fruit.radius)
        .user_data(fruit.kind as u128)
        .build();
    let fruit_rigid_body = RigidBodyBuilder::dynamic()
        .gravity_scale(gravity_scale)
        .ccd_enabled(true)
        .user_data(fruit.kind as u128)
        .build();
    let fruit_rigid_body_handle = rigid_body_set.insert(fruit_rigid_body);
    collider_set.insert_with_parent(fruit_collider, fruit_rigid_body_handle, rigid_body_set);
    fruit_rigid_body_handle
}

fn move_current_fruit(
    rigid_body_set: &mut RigidBodySet,
    current_fruit_handle: Option<RigidBodyHandle>,
) {
    let Some(fruit_handle) = current_fruit_handle else {
        return;
    };
    let current_fruit = rigid_body_set.get_mut(fruit_handle).unwrap();
    current_fruit.set_translation(
        Vector::new((mouse_position().0 / 200.0).clamp(0.80, 2.20), 0.25),
        true,
    );
}

fn drop_current_fruit(
    rigid_body_set: &mut RigidBodySet,
    current_fruit_handle: &mut Option<RigidBodyHandle>,
    drop_time: &mut f64,
) {
    let Some(fruit_handle) = current_fruit_handle else {
        return;
    };
    let current_fruit = rigid_body_set.get_mut(*fruit_handle).unwrap();
    if is_mouse_button_pressed(MouseButton::Left) {
        *drop_time = get_time();
        current_fruit.set_gravity_scale(1.0, true);
        *current_fruit_handle = None;
    }
}

fn respawn_current_fruit(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    current_fruit_handle: &mut Option<RigidBodyHandle>,
    next_fruit: &mut Fruit,
    drop_time: f64,
) {
    if current_fruit_handle.is_none() && get_time() - drop_time > 0.5 {
        *current_fruit_handle = Some(create_fruit(rigid_body_set, collider_set, 0.0, *next_fruit));
        *next_fruit = Fruit::new(rand::gen_range(1, 5));
    }
}

fn collide_fruit(
    island_manager: &mut IslandManager,
    narrow_phase: &NarrowPhase,
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    impulse_joint_set: &mut ImpulseJointSet,
    multibody_joint_set: &mut MultibodyJointSet,
    current_fruit_handle: Option<RigidBodyHandle>,
    score: &mut u32,
    points: &mut u32,
    points_time: &mut f64,
) {
    for contact_pair in narrow_phase.contact_pairs() {
        let mut collider1 = collider_set.get(contact_pair.collider1).unwrap();
        let mut collider2 = collider_set.get(contact_pair.collider2).unwrap();
        if collider1.user_data != collider2.user_data
            || !contact_pair.has_any_active_contact()
            || collider1.user_data == 11
            || collider2.user_data == 11
        {
            continue;
        }
        if let Some(current_fruit_handle) = current_fruit_handle {
            let rigid_body_handle1 = collider1.parent().unwrap();
            let rigid_body_handle2 = collider2.parent().unwrap();
            if current_fruit_handle == rigid_body_handle1
                || current_fruit_handle == rigid_body_handle2
            {
                continue;
            }
        }

        let new_position = (collider1.translation() + collider2.translation()) / 2.0;
        let mut fruit = Fruit::new(collider1.user_data as usize);
        *points = fruit.points;
        *score += *points;
        *points_time = get_time();
        fruit = Fruit::new(collider1.user_data as usize + 1);
        let new_fruit_handle = create_fruit(rigid_body_set, collider_set, 1.0, fruit);
        let new_fruit = rigid_body_set.get_mut(new_fruit_handle).unwrap();
        new_fruit.set_translation(new_position, true);

        collider1 = collider_set.get(contact_pair.collider1).unwrap();
        rigid_body_set.remove(
            collider1.parent().unwrap(),
            island_manager,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            true,
        );

        collider2 = collider_set.get(contact_pair.collider2).unwrap();
        rigid_body_set.remove(
            collider2.parent().unwrap(),
            island_manager,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            true,
        );

        break;
    }
}

fn check_game_over(rigid_body_set: &RigidBodySet) -> bool {
    for rigid_body in rigid_body_set.iter() {
        if rigid_body.1.translation().y > 3.0 {
            return true;
        }
    }
    false
}

fn draw_fruits(rigid_body_set: &RigidBodySet) {
    for rigid_body in rigid_body_set.iter() {
        if rigid_body.1.user_data == 0 {
            continue;
        }
        let fruit = Fruit::new(rigid_body.1.user_data as usize);
        draw_circle(
            rigid_body.1.position().translation.x * 200.0,
            rigid_body.1.position().translation.y * 200.0,
            fruit.radius * 200.0,
            fruit.color,
        );
    }
}

fn draw_container() {
    draw_line(200.0, 400.0, 400.0, 400.0, 10.0, DARKGRAY);
    draw_line(150.0, 100.0, 200.0, 405.0, 10.0, DARKGRAY);
    draw_line(450.0, 100.0, 400.0, 405.0, 10.0, DARKGRAY);
}

fn draw_next_fruit(next_fruit: Fruit) {
    draw_text("Next:", screen_width() - 120.0, 35.0, 30.0, DARKGRAY);
    draw_circle(
        screen_width() - 27.5,
        27.5,
        next_fruit.radius * 200.0,
        next_fruit.color,
    );
}

fn draw_score(score: &mut u32, points: u32, points_time: &mut f64) {
    draw_text(&format!("Score: {}", score), 15.0, 35.0, 30.0, DARKGRAY);
    let score_len = score.checked_ilog10().unwrap_or(0) + 1;
    let points_len = points.checked_ilog10().unwrap_or(0) + 1;
    if get_time() - *points_time < 2.0 {
        draw_text(
            &format!(
                "     {}+{}",
                " ".repeat((score_len - points_len + 1) as usize),
                points
            ),
            15.0,
            65.0,
            30.0,
            DARKGRAY,
        );
    }
}
