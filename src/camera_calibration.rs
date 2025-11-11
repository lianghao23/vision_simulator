use bevy::input::keyboard::KeyCode;
use bevy::prelude::*;
use std::fs::File;
use std::io::Write;

/// Component to mark chessboard for calibration
#[derive(Component)]
pub struct ChessboardCalibration {
    pub rows: u32,
    pub cols: u32,
    pub square_size: f32,
}

/// Resource to store virtual calibration results
#[derive(Resource, Default)]
pub struct VirtualCalibrationResults {
    pub chessboard_detected: bool,
    pub image_points: Vec<Vec2>,
    pub object_points: Vec<Vec3>,
    pub camera_matrix: Option<CameraMatrix>,
    pub distortion_coefficients: Option<DistortionCoefficients>,
}

/// Camera calibration parameters
#[derive(Resource, Debug, Clone)]
pub struct CameraCalibration {
    pub image_width: u32,
    pub image_height: u32,
    pub camera_name: String,
    pub camera_matrix: CameraMatrix,
    pub distortion_model: String,
    pub distortion_coefficients: DistortionCoefficients,
    pub rectification_matrix: RectificationMatrix,
    pub projection_matrix: ProjectionMatrix,
}

#[derive(Debug, Clone)]
pub struct CameraMatrix {
    pub rows: u32,
    pub cols: u32,
    pub data: [f32; 9],
}

#[derive(Debug, Clone)]
pub struct DistortionCoefficients {
    pub rows: u32,
    pub cols: u32,
    pub data: [f32; 5],
}

#[derive(Debug, Clone)]
pub struct RectificationMatrix {
    pub rows: u32,
    pub cols: u32,
    pub data: [f32; 9],
}

#[derive(Debug, Clone)]
pub struct ProjectionMatrix {
    pub rows: u32,
    pub cols: u32,
    pub data: [f32; 12],
}

/// System to calculate camera calibration parameters
pub fn calculate_camera_calibration(
    windows: Query<&Window>,
    camera_query: Query<(&Camera, &GlobalTransform, &Projection)>,
    mut calibration: ResMut<CameraCalibration>,
) {
    if let (Ok(window), Ok((camera, camera_transform, projection))) =
        (windows.single(), camera_query.single())
    {
        // Update image dimensions
        calibration.image_width = window.resolution.width() as u32;
        calibration.image_height = window.resolution.height() as u32;

        // Calculate camera matrix (intrinsic parameters)
        calculate_camera_matrix(&mut calibration.camera_matrix, camera, projection, window);

        // For simulation, distortion coefficients are typically zero or minimal
        calibration.distortion_coefficients.data = [0.0, 0.0, 0.0, 0.0, 0.0];

        // Rectification matrix is identity for monocular cameras
        calibration.rectification_matrix.data = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

        // Calculate projection matrix
        calculate_projection_matrix(
            &mut calibration.projection_matrix,
            camera,
            projection,
            camera_transform,
        );
    }
}

fn calculate_camera_matrix(
    camera_matrix: &mut CameraMatrix,
    camera: &Camera,
    projection: &Projection,
    window: &Window,
) {
    let (width, height) = (window.width(), window.height());

    match projection {
        Projection::Perspective(perspective) => {
            let fov_x = perspective.fov; // In Bevy, FOV is typically horizontal
            let aspect_ratio = width / height;

            // Focal lengths in pixels
            let fx = (width as f32) / (2.0 * (fov_x / 2.0).tan());
            let fy = fx / aspect_ratio;

            // Principal point (image center)
            let cx = width / 2.0;
            let cy = height / 2.0;

            camera_matrix.data = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0];
        }
        Projection::Orthographic(orthographic) => {
            // For orthographic projection, use window dimensions
            let fx = width / (orthographic.area.width());
            let fy = height / (orthographic.area.height());
            let cx = width / 2.0;
            let cy = height / 2.0;

            camera_matrix.data = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0];
        }
        _ => todo!(),
    }
}

fn calculate_projection_matrix(
    projection_matrix: &mut ProjectionMatrix,
    camera: &Camera,
    projection: &Projection,
    camera_transform: &GlobalTransform,
) {
    match projection {
        Projection::Perspective(perspective) => {
            let (width, height) = if let Some(viewport) = &camera.viewport {
                (
                    viewport.physical_size.x as f32,
                    viewport.physical_size.y as f32,
                )
            } else {
                // Fallback to reasonable defaults
                (1440.0, 1080.0)
            };

            let fov_x = perspective.fov; // In Bevy, FOV is typically horizontal
            let aspect_ratio = width / height;
            let near = perspective.near;
            let far = perspective.far;

            let fx = (width as f32) / (2.0 * (fov_x / 2.0).tan());
            let fy = fx / aspect_ratio;
            let cx = width / 2.0;
            let cy = height / 2.0;

            // ROS-style projection matrix: [fx' 0 cx' Tx; 0 fy' cy' Ty; 0 0 1 0]
            // where fx' = fx, fy' = fy, and Tx = -fx * baseline_x, Ty = -fy * baseline_y
            // For monocular cameras, baseline is 0, so Tx = Ty = 0
            projection_matrix.data = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0];
        }
        Projection::Orthographic(orthographic) => {
            let left = orthographic.area.min.x;
            let right = orthographic.area.max.x;
            let bottom = orthographic.area.min.y;
            let top = orthographic.area.max.y;
            let near = orthographic.near;
            let far = orthographic.far;

            // For orthographic projection in ROS format
            let fx = (right - left) / 2.0;
            let fy = (top - bottom) / 2.0;
            let cx = (right + left) / 2.0;
            let cy = (top + bottom) / 2.0;

            projection_matrix.data = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0];
        }
        _ => todo!(),
    }
}

/// System to save calibration data to YAML file
pub fn save_calibration_to_file(
    keys: Res<ButtonInput<KeyCode>>,
    calibration: Res<CameraCalibration>,
) {
    if keys.just_pressed(KeyCode::KeyS) {
        let filename = "camera_calibration.yaml";
        match File::create(filename) {
            Ok(mut file) => {
                let yaml_content = format!(
                    "image_width: {}\n\
                     image_height: {}\n\
                     camera_name: {}\n\
                     camera_matrix:\n\
                       rows: {}\n\
                       cols: {}\n\
                       data: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}]\n\
                     distortion_model: {}\n\
                     distortion_coefficients:\n\
                       rows: {}\n\
                       cols: {}\n\
                       data: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}]\n\
                     rectification_matrix:\n\
                       rows: {}\n\
                       cols: {}\n\
                       data: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}]\n\
                     projection_matrix:\n\
                       rows: {}\n\
                       cols: {}\n\
                       data: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}]",
                    calibration.image_width,
                    calibration.image_height,
                    calibration.camera_name,
                    calibration.camera_matrix.rows,
                    calibration.camera_matrix.cols,
                    calibration.camera_matrix.data[0],
                    calibration.camera_matrix.data[1],
                    calibration.camera_matrix.data[2],
                    calibration.camera_matrix.data[3],
                    calibration.camera_matrix.data[4],
                    calibration.camera_matrix.data[5],
                    calibration.camera_matrix.data[6],
                    calibration.camera_matrix.data[7],
                    calibration.camera_matrix.data[8],
                    calibration.distortion_model,
                    calibration.distortion_coefficients.rows,
                    calibration.distortion_coefficients.cols,
                    calibration.distortion_coefficients.data[0],
                    calibration.distortion_coefficients.data[1],
                    calibration.distortion_coefficients.data[2],
                    calibration.distortion_coefficients.data[3],
                    calibration.distortion_coefficients.data[4],
                    calibration.rectification_matrix.rows,
                    calibration.rectification_matrix.cols,
                    calibration.rectification_matrix.data[0],
                    calibration.rectification_matrix.data[1],
                    calibration.rectification_matrix.data[2],
                    calibration.rectification_matrix.data[3],
                    calibration.rectification_matrix.data[4],
                    calibration.rectification_matrix.data[5],
                    calibration.rectification_matrix.data[6],
                    calibration.rectification_matrix.data[7],
                    calibration.rectification_matrix.data[8],
                    calibration.projection_matrix.rows,
                    calibration.projection_matrix.cols,
                    calibration.projection_matrix.data[0],
                    calibration.projection_matrix.data[1],
                    calibration.projection_matrix.data[2],
                    calibration.projection_matrix.data[3],
                    calibration.projection_matrix.data[4],
                    calibration.projection_matrix.data[5],
                    calibration.projection_matrix.data[6],
                    calibration.projection_matrix.data[7],
                    calibration.projection_matrix.data[8],
                    calibration.projection_matrix.data[9],
                    calibration.projection_matrix.data[10],
                    calibration.projection_matrix.data[11]
                );

                if let Err(e) = writeln!(file, "{}", yaml_content) {
                    println!("Failed to write calibration file: {}", e);
                } else {
                    println!("Calibration data saved to {}", filename);
                }
            }
            Err(e) => {
                println!("Failed to create calibration file: {}", e);
            }
        }
    }
}

/// System to print calibration data to console
pub fn print_calibration_data(calibration: Res<CameraCalibration>) {
    println!("=== Camera Calibration Data ===");
    println!("image_width: {}", calibration.image_width);
    println!("image_height: {}", calibration.image_height);
    println!("camera_name: {}", calibration.camera_name);
    println!("camera_matrix:");
    println!("  rows: {}", calibration.camera_matrix.rows);
    println!("  cols: {}", calibration.camera_matrix.cols);
    println!("  data: {:?}", calibration.camera_matrix.data);
    println!("distortion_model: {}", calibration.distortion_model);
    println!("distortion_coefficients:");
    println!("  rows: {}", calibration.distortion_coefficients.rows);
    println!("  cols: {}", calibration.distortion_coefficients.cols);
    println!("  data: {:?}", calibration.distortion_coefficients.data);
    println!("rectification_matrix:");
    println!("  rows: {}", calibration.rectification_matrix.rows);
    println!("  cols: {}", calibration.rectification_matrix.cols);
    println!("  data: {:?}", calibration.rectification_matrix.data);
    println!("projection_matrix:");
    println!("  rows: {}", calibration.projection_matrix.rows);
    println!("  cols: {}", calibration.projection_matrix.cols);
    println!("  data: {:?}", calibration.projection_matrix.data);
    println!("===============================");
}

/// System to handle keyboard input for calibration output
pub fn calibration_keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    calibration: Res<CameraCalibration>,
) {
    if keys.just_pressed(KeyCode::KeyC) {
        println!("=== Camera Calibration Data (Press C) ===");
        println!("image_width: {}", calibration.image_width);
        println!("image_height: {}", calibration.image_height);
        println!("camera_name: {}", calibration.camera_name);
        println!("camera_matrix:");
        println!("  rows: {}", calibration.camera_matrix.rows);
        println!("  cols: {}", calibration.camera_matrix.cols);
        println!(
            "  data: [{}]",
            calibration
                .camera_matrix
                .data
                .iter()
                .map(|x| format!("{:.3}", x))
                .collect::<Vec<_>>()
                .join(", ")
        );
        println!("distortion_model: {}", calibration.distortion_model);
        println!("distortion_coefficients:");
        println!("  rows: {}", calibration.distortion_coefficients.rows);
        println!("  cols: {}", calibration.distortion_coefficients.cols);
        println!(
            "  data: [{}]",
            calibration
                .distortion_coefficients
                .data
                .iter()
                .map(|x| format!("{:.3}", x))
                .collect::<Vec<_>>()
                .join(", ")
        );
        println!("rectification_matrix:");
        println!("  rows: {}", calibration.rectification_matrix.rows);
        println!("  cols: {}", calibration.rectification_matrix.cols);
        println!(
            "  data: [{}]",
            calibration
                .rectification_matrix
                .data
                .iter()
                .map(|x| format!("{:.3}", x))
                .collect::<Vec<_>>()
                .join(", ")
        );
        println!("projection_matrix:");
        println!("  rows: {}", calibration.projection_matrix.rows);
        println!("  cols: {}", calibration.projection_matrix.cols);
        println!(
            "  data: [{}]",
            calibration
                .projection_matrix
                .data
                .iter()
                .map(|x| format!("{:.3}", x))
                .collect::<Vec<_>>()
                .join(", ")
        );
        println!("===============================");
    }
}

/// System to setup chessboard for calibration
pub fn setup_chessboard_calibration(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Spawn the CALIB.glb chessboard with calibration component
    commands.spawn((
        SceneRoot(asset_server.load("CALIB.glb#Scene0")),
        Transform::IDENTITY
            .with_scale(Vec3::splat(1.0))
            .with_translation(Vec3::new(1.0, 0.5, 1.0)),
        ChessboardCalibration {
            rows: 8,           // Typical chessboard pattern
            cols: 11,          // Typical chessboard pattern
            square_size: 0.15, // Adjust based on your CALIB.glb dimensions
        },
    ));
}

/// System to perform virtual chessboard calibration
pub fn virtual_chessboard_calibration(
    mut calibration_results: ResMut<VirtualCalibrationResults>,
    chessboard_query: Query<(&Transform, &ChessboardCalibration)>,
    camera_query: Query<(&Camera, &GlobalTransform, &Projection)>,
    windows: Query<&Window>,
) {
    if let (
        Ok(window),
        Ok((camera, camera_transform, projection)),
        Ok((chessboard_transform, chessboard)),
    ) = (
        windows.single(),
        camera_query.single(),
        chessboard_query.single(),
    ) {
        // Project chessboard corners to image space
        let mut image_points = Vec::new();
        let mut object_points = Vec::new();

        // Generate chessboard corners in 3D space
        for row in 0..chessboard.rows {
            for col in 0..chessboard.cols {
                let x =
                    (col as f32 - (chessboard.cols as f32 - 1.0) / 2.0) * chessboard.square_size;
                let y =
                    (row as f32 - (chessboard.rows as f32 - 1.0) / 2.0) * chessboard.square_size;
                let z = 0.0;

                let world_point = chessboard_transform.transform_point(Vec3::new(x, y, z));
                object_points.push(world_point);

                // Project to image space
                if let Some(ndc) = camera.world_to_ndc(camera_transform, world_point) {
                    let pixel_pos = ndc_to_pixel(ndc, window);
                    image_points.push(pixel_pos);
                }
            }
        }

        calibration_results.image_points = image_points;
        calibration_results.object_points = object_points;
        calibration_results.chessboard_detected = !calibration_results.image_points.is_empty();

        // If we have enough points, calculate camera matrix
        if calibration_results.image_points.len() >= 6 {
            calculate_virtual_camera_matrix(&mut calibration_results, camera, projection, window);
        }
    }
}

/// Convert normalized device coordinates to pixel coordinates
fn ndc_to_pixel(ndc: Vec3, window: &Window) -> Vec2 {
    let width = window.width();
    let height = window.height();

    Vec2::new(
        (ndc.x + 1.0) * 0.5 * width,
        (1.0 - ndc.y) * 0.5 * height, // Flip Y for image coordinates
    )
}

/// Calculate camera matrix from virtual chessboard points
fn calculate_virtual_camera_matrix(
    results: &mut VirtualCalibrationResults,
    camera: &Camera,
    projection: &Projection,
    window: &Window,
) {
    let (width, height) = (window.width(), window.height());

    match projection {
        Projection::Perspective(perspective) => {
            let fov_x = perspective.fov; // In Bevy, FOV is typically horizontal
            let aspect_ratio = width / height;

            // Focal lengths in pixels
            let fx = (width as f32) / (2.0 * (fov_x / 2.0).tan());
            let fy = fx / aspect_ratio;

            // Principal point (image center)
            let cx = width / 2.0;
            let cy = height / 2.0;

            results.camera_matrix = Some(CameraMatrix {
                rows: 3,
                cols: 3,
                data: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            });
        }
        Projection::Orthographic(_) => {
            // For orthographic, use simpler calculation
            results.camera_matrix = Some(CameraMatrix {
                rows: 3,
                cols: 3,
                data: [
                    width,
                    0.0,
                    width / 2.0,
                    0.0,
                    height,
                    height / 2.0,
                    0.0,
                    0.0,
                    1.0,
                ],
            });
        }
        _ => todo!(),
    }

    // For simulation, distortion is minimal
    results.distortion_coefficients = Some(DistortionCoefficients {
        rows: 1,
        cols: 5,
        data: [0.0, 0.0, 0.0, 0.0, 0.0],
    });
}

/// System to print virtual calibration results
pub fn print_virtual_calibration(results: Res<VirtualCalibrationResults>) {
    if results.chessboard_detected {
        println!("=== Virtual Chessboard Calibration ===");
        println!("Chessboard detected: {}", results.chessboard_detected);
        println!("Number of detected corners: {}", results.image_points.len());

        if let Some(camera_matrix) = &results.camera_matrix {
            println!("Camera Matrix:");
            println!(
                "  fx: {:.3}, fy: {:.3}",
                camera_matrix.data[0], camera_matrix.data[4]
            );
            println!(
                "  cx: {:.3}, cy: {:.3}",
                camera_matrix.data[2], camera_matrix.data[5]
            );
        }

        if let Some(dist_coeffs) = &results.distortion_coefficients {
            println!("Distortion Coefficients:");
            println!(
                "  k1: {:.3}, k2: {:.3}, p1: {:.3}, p2: {:.3}, k3: {:.3}",
                dist_coeffs.data[0],
                dist_coeffs.data[1],
                dist_coeffs.data[2],
                dist_coeffs.data[3],
                dist_coeffs.data[4]
            );
        }
        println!("=====================================");
    }
}

/// Plugin for camera calibration
pub struct CameraCalibrationPlugin;

impl Plugin for CameraCalibrationPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(CameraCalibration {
            image_width: 1440,
            image_height: 1080,
            camera_name: "camera".to_string(),
            camera_matrix: CameraMatrix {
                rows: 3,
                cols: 3,
                data: [0.0; 9],
            },
            distortion_model: "plumb_bob".to_string(),
            distortion_coefficients: DistortionCoefficients {
                rows: 1,
                cols: 5,
                data: [0.0; 5],
            },
            rectification_matrix: RectificationMatrix {
                rows: 3,
                cols: 3,
                data: [0.0; 9],
            },
            projection_matrix: ProjectionMatrix {
                rows: 3,
                cols: 4,
                data: [0.0; 12],
            },
        })
        .insert_resource(VirtualCalibrationResults::default())
        .add_systems(Startup, setup_chessboard_calibration)
        .add_systems(
            Update,
            (
                calculate_camera_calibration,
                print_calibration_data,
                calibration_keyboard_input,
                save_calibration_to_file,
                virtual_chessboard_calibration,
                print_virtual_calibration,
            ),
        );
    }
}
