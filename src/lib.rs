use ncollide3d::nalgebra::{Isometry3, Point3, Vector3};
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::TriMesh;
use std::convert::TryInto;

#[repr(C)]
pub struct MeshInitResult {
    success: i32,
    mesh: *const TriMesh<f32>,
}

#[no_mangle]
pub extern "C" fn mesh_init(
    vertices_raw_data: *const f32,
    vertices_raw_data_length: i32,
    faces_raw_data: *const u32,
    faces_raw_data_length: i32,
) -> MeshInitResult {
    if vertices_raw_data_length % 3 != 0 {
        return MeshInitResult {
            success: 0,
            mesh: std::ptr::null(),
        };
    }

    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut i = 0;
    while i < vertices_raw_data_length {
        unsafe {
            vertices.push(Point3::new(
                *(vertices_raw_data.offset(i.try_into().unwrap())),
                *(vertices_raw_data.offset((i + 1).try_into().unwrap())),
                *(vertices_raw_data.offset((i + 2).try_into().unwrap())),
            ));
        }
        i += 3;
    }

    let mut triangles: Vec<Point3<usize>> = Vec::new();
    let mut current_face_count_index = 0;
    while current_face_count_index < faces_raw_data_length {
        let count: i32 =
            (unsafe { *(faces_raw_data.offset(current_face_count_index.try_into().unwrap())) })
                .try_into()
                .unwrap();
        let mut current_face_index = current_face_count_index + 1;
        // TODO This only works with convex polygons
        while current_face_index < current_face_count_index + count - 1 {
            triangles.push(Point3::new(
                (unsafe {
                    *faces_raw_data.offset((current_face_count_index + 1).try_into().unwrap())
                } - 1)
                    .try_into()
                    .unwrap(),
                (unsafe { *faces_raw_data.offset((current_face_index + 1).try_into().unwrap()) }
                    - 1)
                .try_into()
                .unwrap(),
                (unsafe { *faces_raw_data.offset((current_face_index + 2).try_into().unwrap()) }
                    - 1)
                .try_into()
                .unwrap(),
            ));
            current_face_index = current_face_index + 1;
        }
        current_face_count_index = current_face_count_index + count + 1;
    }

    return MeshInitResult {
        success: 1,
        mesh: Box::into_raw(Box::new(TriMesh::new(vertices, triangles, None))),
    };
}

#[repr(C)]
pub struct MeshCollideResult {
    has_collision: i32,
    value: f32,
}

#[no_mangle]
pub extern "C" fn mesh_collide(
    p_mesh: *mut TriMesh<f32>,
    origin_x: f32,
    origin_y: f32,
    origin_z: f32,
    direction_x: f32,
    direction_y: f32,
    direction_z: f32,
) -> MeshCollideResult {
    // TODO check max distance
    let direction_vector = Vector3::new(direction_x, direction_y, direction_z);
    let ray = Ray::new(Point3::new(origin_x, origin_y, origin_z), direction_vector);

    return unsafe { p_mesh.as_ref() }
        .unwrap()
        .toi_with_ray(
            &Isometry3::identity(),
            &ray,
            direction_vector.magnitude(),
            true,
        )
        .map(|ray| MeshCollideResult {
            has_collision: 1,
            value: ray,
        })
        .unwrap_or(MeshCollideResult {
            has_collision: 0,
            value: 0.0f32,
        });
}

#[no_mangle]
pub extern "C" fn mesh_free(mesh: *mut TriMesh<f32>) {
    unsafe { Box::from_raw(mesh) };
}
