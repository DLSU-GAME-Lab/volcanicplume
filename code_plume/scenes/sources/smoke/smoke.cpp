#include "smoke.hpp"


using namespace vcl;



// Counter used to save image on hard drive
int counter_image = 0;

std::chrono::system_clock::time_point a = std::chrono::system_clock::now();
std::chrono::system_clock::time_point b = std::chrono::system_clock::now();



//------------------------------------------------------------
//------------------------ TERRAIN ---------------------------
//------------------------------------------------------------

void scene_model::fill_height_field(vcl::buffer<vcl::vec3>& position, vcl::buffer<vcl::vec3>& normal,
                                    vcl::mesh_drawable terrain)
{
    // prepare field with parameters
    terrain_struct.min_xyz = -15000.0f;
    terrain_struct.max_xyz = 15000.0f;
    float interval_size = terrain_struct.max_xyz - terrain_struct.min_xyz;
    terrain_struct.cell_size = 200.f;
    terrain_struct.field_size = (size_t)(terrain_struct.max_xyz/terrain_struct.cell_size);
    terrain_struct.height_field.resize(terrain_struct.field_size, terrain_struct.field_size);
    terrain_struct.normal_field.resize(terrain_struct.field_size, terrain_struct.field_size);

    //transform like for mesh_drawable
    mat3 rotation = terrain.uniform.transform.rotation;
    vec3 translation = terrain.uniform.transform.translation;
    float scaling = terrain.uniform.transform.scaling;
    for (unsigned int i = 0; i<position.size(); i++)
    {
        position[i] = scaling * (rotation * position[i] + translation);
        normal[i] = rotation * normal[i];
    }
    terrain_struct.positions = position;
    terrain_struct.normals = normal;


    //fill height field for collisions
    for (unsigned int i = 0; i<position.size(); i++)
    {
        if (position[i].x < terrain_struct.max_xyz && position[i].x > terrain_struct.min_xyz
                && position[i].y < terrain_struct.max_xyz && position[i].y > terrain_struct.min_xyz)
        {
            int idx_x = (int)(terrain_struct.field_size*(position[i].x - terrain_struct.min_xyz)/interval_size);
            int idx_y = (int)(terrain_struct.field_size*(position[i].y - terrain_struct.min_xyz)/interval_size);
            if (idx_x == terrain_struct.field_size) idx_x = terrain_struct.field_size-1;
            if (idx_y == terrain_struct.field_size) idx_y = terrain_struct.field_size-1;
            terrain_struct.height_field(idx_x,idx_y) = position[i].z;
            terrain_struct.normal_field(idx_x,idx_y) = normal[i];
        }
    }

}


//------------------------------------------------------------
//------------------------ PROCESS ---------------------------
//------------------------------------------------------------ */

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    // Maintain designated frequency of 5 Hz (200 ms per frame)
//    a = std::chrono::system_clock::now();
//    std::chrono::duration<double, std::milli> work_time = a - b;

//    if (work_time.count() < 16.0)
//    {
//        std::chrono::duration<double, std::milli> delta_ms(16.0 - work_time.count());
//        auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
//        std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
//    }

//    b = std::chrono::system_clock::now();
//    std::chrono::duration<double, std::milli> sleep_time = b - a;

    const float dt = timer.update();
    set_gui();

    // Force constant time step
    t_step = dt<=1e-6f? 0.0f : timer.scale*0.002f; //0.0003f
    new_layer_delay += t_step;


    if (!replay)
    {
        for (unsigned int nb_steps_per_frame = 0; nb_steps_per_frame<10; nb_steps_per_frame++)
        {
            // add smoke layer each x seconds
            if (smoke_layers.size() == 0 || (new_layer_delay >= r_0/(2*U_0) && smoke_layers.size() < 1000000000000000))
            {
                add_smoke_layer(U_0, rho_0, r_0, vec3(0,0,z_0), false);
                add_free_spheres_for_one_layer(smoke_layers.size()-1);

                new_layer_delay = 0;
                total_layers_ejected++;
                std::cout << smoke_layers.size() << std::endl;
                if (debug_mode) std::cout << "LAYER ADDED OK" << std::endl;
            }

            // update of layer and spheres
            for (unsigned int id=0; id<smoke_layers.size(); id++)
            {
                smoke_layer_update(id);
            }
            update_free_spheres();
            if (frame_count %100 == 0) falling_spheres_update(100);
            update_stagnation_spheres();

            // update subspheres
            //if (frame_count %50 == 0) update_subspheres_params();

            // export (comment or uncomment)
            if (export_data && frame_count % 50 == 0) export_spheres();

            // store data for replay
            if (!export_data && frame_count %50 == 0)
            {
                smoke_layers_frames.push_back(smoke_layers);
                free_spheres_frames.push_back(free_spheres);
                stagnate_spheres_frames.push_back(stagnate_spheres);
                falling_spheres_frames.push_back(falling_spheres);
                falling_spheres_buffers_frames.push_back(falling_spheres_buffers);
            }

            frame_count++;
        }
    }

    if (replay)
    {
        display_replay(shaders, scene, gui, frame_replay);
        std::cout << frame_replay << std::endl;
        frame_replay++;
        frame_replay = frame_replay % smoke_layers_frames.size();
    }
    else display(shaders, scene, gui);
}


//------------------------------------------------------------
//------------------------- ALGO -----------------------------
//------------------------------------------------------------ */

void scene_model::add_smoke_layer(float v, float d, float r, vec3 position, bool secondary_plume)
{
    smoke_layer layer = smoke_layer({0,0,v}, d, r, position, secondary_plume);
    smoke_layers.push_back(layer);
}

float compute_atm_temperature(float height) //(K) cf https://fr.wikipedia.org/wiki/Atmosph%C3%A8re_normalis%C3%A9e
{
    return 288.15 - 6.5*height/1000.0;
}

float compute_atm_density(float height) //(kg.m-3) cf https://www.deleze.name/marcel/sec2/applmaths/pression-altitude/masse_volumique.pdf
{
    return 352.995 * pow(1 - 0.0000225577*height, 5.25516) / (288.15 - 0.0065*height);
}

vcl::vec3 scene_model::compute_wind_vector(float height)
{
    // find altitude interval
    unsigned int low_altitude_idx = 0;
    for (unsigned int i = 0; i<wind_altitudes.size(); i++)
    {
        if (wind_altitudes[i] < height)
        {
            low_altitude_idx = i;
        }
    }

    // compute wind vec by interpolating
    if (low_altitude_idx == wind_altitudes.size()-1)
    {
        return winds[low_altitude_idx].wind_vector;
    }
    else
    {
        float low_height = (float)wind_altitudes[low_altitude_idx];
        float high_height = (float)wind_altitudes[low_altitude_idx+1];
        float lambda = (height-low_height)/(high_height-low_height);
        vec3 interpo_wind = winds[low_altitude_idx].wind_vector + lambda * (winds[low_altitude_idx+1].wind_vector - winds[low_altitude_idx].wind_vector);
        return interpo_wind;
    }
}

void scene_model::edit_smoke_layer_properties(unsigned int i, float& d_mass)
{
    // get wind at altitude
    vec3 wind = compute_wind_vector(smoke_layers[i].center.z);

    // preliminary computation
    float thk = t_step * smoke_layers[i].v.z;
    //thk = smoke_layers[i].v.z * 0.002;

    float total_smoke_thk = smoke_layers[i].thickness;
    float total_smoke_volume = total_smoke_thk * 3.14 * smoke_layers[i].r * smoke_layers[i].r;
    float total_smoke_mass = smoke_layers[i].rho * total_smoke_volume;

    // wind velocity around for air incorporation
    float k_s = 0.09, k_w = 0.9;
    float U_e = k_s * abs(norm(smoke_layers[i].v) - norm(wind)*cos(smoke_layers[i].theta)) + k_w * abs(norm(wind)*sin(smoke_layers[i].theta));
    float r_atm = air_incorporation_coeff * U_e;
    //r_atm = U_e * t_step;

    // air quantity to put in the plume
    float atm_density = compute_atm_density(smoke_layers[i].center.z);
    float atm_volume = thk * 3.14 * (2.0 * smoke_layers[i].r + r_atm) * r_atm; //air around the smoke layer
    float atm_mass = atm_density * atm_volume;

    // compute new temperature (we take all Cp equal)
    float new_temp = (total_smoke_mass * smoke_layers[i].temperature + atm_mass * compute_atm_temperature(smoke_layers[i].center.z)) / (total_smoke_mass + atm_mass);
    smoke_layers[i].temperature = new_temp;

    // new volume after heating
    float atm_new_volume = smoke_layers[i].temperature * atm_volume / compute_atm_temperature(smoke_layers[i].center.z); // after heating by hot smoke

    // new params
    d_mass = atm_mass;
    float mass_new = total_smoke_mass + atm_mass;
    float volume_new = total_smoke_volume + atm_new_volume;
    float rho_new = mass_new/volume_new;
    float r_new = cbrt(volume_new/3.14);
    smoke_layers[i].thickness = r_new;


    // new speed due to conservation of energy (old)
    //float energy = 0.5 * total_smoke_mass * smoke_layers[i].v.z * smoke_layers[i].v.z;
    //float new_speed = sqrt(2.0 * energy / mass_new);
    //if (rho_new < atm_density) smoke_layers[i].v = {0,0,new_speed};

    if (smoke_layers[i].rho > atm_density && rho_new < atm_density) smoke_layers[i].plume = true;
    smoke_layers[i].r = r_new;
    smoke_layers[i].rho = rho_new;
}

void scene_model::apply_forces_to_smoke_layer(unsigned int i, float d_mass)
{
    // get wind at altitude
    vec3 wind = compute_wind_vector(smoke_layers[i].center.z);

    // precomputation
    float atm_density = compute_atm_density(smoke_layers[i].center.z);
    float volume = smoke_layers[i].thickness*3.14*smoke_layers[i].r*smoke_layers[i].r;
    float surface = 2*3.14*smoke_layers[i].r*smoke_layers[i].r;
    float surface_eff = 2*smoke_layers[i].r*smoke_layers[i].r;
    float smoke_mass = smoke_layers[i].rho * volume;
    vec3 v_diff = wind - vec3(smoke_layers[i].v.x, smoke_layers[i].v.y,0);

    vec3 weight = {0,0, -smoke_mass * g}; // m*g
    vec3 archimede = {0,0, atm_density * volume * g}; // rho*V*g
    vec3 friction = - 0.5 * atm_density * 0.04 * surface * norm(smoke_layers[i].v) * smoke_layers[i].v; // axial friction
    //friction = - volume * 0.00005 * norm(smoke_layers[i].v) * smoke_layers[i].v; // old way to compute friction
    vec3 wind_force = 600. * norm(v_diff)*v_diff * surface_eff; // horizontal

    vec3 forces = weight + archimede + friction + wind_force;

    vec3 old_v = smoke_layers[i].v;
    float old_m = smoke_mass - d_mass;

    // equation of dynamics without mass conservation
    vec3 mv = old_m * old_v + forces*t_step;
    vec3 v = mv/smoke_mass;
    vec3 p = smoke_layers[i].center + v*t_step;

    // old with mass conservation
    //vec3 a = forces / (smoke_mass);
    //vec3 v = smoke_layers[i].v + a*t_step;
    //vec3 p = smoke_layers[i].center + v*t_step;


    // check if falls
    if (old_v.z >0 && v.z < 0 && smoke_layers[i].plume == false)
    {
        smoke_layers[i].rising = false;
        smoke_layers[i].begin_falling = true;
        std::cout << "layer " << i << " falling frame " << frame_count << std::endl;
    }

    // check if stagnates
    if (smoke_layers[i].plume && !smoke_layers[i].stagnates && smoke_layers[i].center.z > 5000 && (atm_density - smoke_layers[i].rho < 0.01))
    {
        smoke_layers[i].stagnates = true;
        stagnation_speed = smoke_layers[i].v.z;
    }

    // check if stagnates long
    if (smoke_layers[i].stagnates && v.z < 0 && !smoke_layers[i].stagnates_long)
    {
        smoke_layers[i].stagnates_long = true;
    }

    if (smoke_layers[i].stagnates && p.z < smoke_layers[i].center.z)
    {
        p.z = smoke_layers[i].center.z;
    }

    // update
    smoke_layers[i].a = v/t_step;
    smoke_layers[i].v = v;
    smoke_layers[i].center = p;
    if (p.z < -1000) smoke_layers[i].center.z = -1000; // prevent from going oob after falling (layers are not deleted but not used anymore)

    // compute new theta
    // WARNING: wind direction was constant in my tests, if the direction changes it may not work, it has to be tested
    //float dx = v.x*t_step, dy = v.y*t_step;
    float dz = v.z*t_step;
    smoke_layers[i].speed_along_axis = norm(v);
    smoke_layers[i].plume_axis = normalize(v);
    if (norm(wind) != 0 && t_step > 0)
    {
        smoke_layers[i].theta_axis = normalize(cross(wind, vec3(0,0,1)));
        float theta_totest = asin(dz/norm(v*t_step));
        if (dz/norm(v*t_step) < 1.000001 && dz/norm(v*t_step) > 0.999999) theta_totest = 3.14/2.; // security to prevent nan values due to asin
        smoke_layers[i].theta = theta_totest;

        if (theta_totest < 0)
        {
            smoke_layers[i].theta = 0;
        }
    }


    // print values in txt files
    nb_of_iterations++;
    if(nb_of_iterations%10 == 1)
    {
        if (altitude) altitude << smoke_layers[i].center.z << std::endl;
        if (plumex) plumex << smoke_layers[i].center.x << std::endl;
        if (speed) speed << smoke_layers[i].v.z << std::endl;
        if (ray) ray << smoke_layers[i].r << std::endl;
        if (smoke_rho) smoke_rho << smoke_layers[i].rho << std::endl;
        if (atm_rho) atm_rho << atm_density << std::endl;
        if (temp) temp << smoke_layers[i].temperature << std::endl;
    }

}

void scene_model::sedimentation(unsigned int i, float& d_mass)
{
    // constant sedimentation
    float layer_volume = 3.14 * smoke_layers[i].r*smoke_layers[i].r * smoke_layers[i].thickness;
    float diff_density = 0.00000005 * t_step;
    if (smoke_layers[i].stagnates_long) diff_density = 0.00005 * t_step;

    if (smoke_layers[i].rho > diff_density)
    {
        smoke_layers[i].rho -= diff_density;
        d_mass -= diff_density * layer_volume;
    }

}

void scene_model::smoke_layer_update(unsigned int i)
{
    float d_mass = 0; // to track mass change for equation of dynamics
    if (smoke_layers[i].plume == true && smoke_layers[i].center.z > 0.) sedimentation(i, d_mass); // sedimentation in altitude
    if (smoke_layers[i].rising && !smoke_layers[i].stagnates_long) edit_smoke_layer_properties(i, d_mass); // convection if v_z > 0 (convection causes air entrainment)
    apply_forces_to_smoke_layer(i, d_mass);
}


//------------------------------------------------------------
//--------------------- FALLING SPHERES ----------------------
//------------------------------------------------------------ */

float scene_model::field_height_at(float x, float y)
{
    float interval_size = terrain_struct.max_xyz - terrain_struct.min_xyz;
    int idx_x = (int)(terrain_struct.field_size*(x - terrain_struct.min_xyz)/interval_size);
    int idx_y = (int)(terrain_struct.field_size*(y - terrain_struct.min_xyz)/interval_size);
    return terrain_struct.height_field(idx_x,idx_y);
}

vcl::vec3 scene_model::field_normal_at(float x, float y)
{
    float interval_size = terrain_struct.max_xyz - terrain_struct.min_xyz;
    int idx_x = (int)(terrain_struct.field_size*(x - terrain_struct.min_xyz)/interval_size);
    int idx_y = (int)(terrain_struct.field_size*(y - terrain_struct.min_xyz)/interval_size);
    return terrain_struct.normal_field(idx_x,idx_y);
}

void scene_model::sphere_ground_collision(free_sphere_params& sphere, int idx, unsigned int frame_nb)
{
    // find ground point and normal
    float terrain_z = field_height_at(sphere.center.x, sphere.center.y);

    // if sphere under ground mesh
    if (sphere.center.z < terrain_z)
    {

        vec3 terrain_pt(sphere.center.x, sphere.center.y, terrain_z);
        vec3 terrain_normal = field_normal_at(sphere.center.x, sphere.center.y);
        vec3 pt_diff = terrain_pt - sphere.center;

        // compute new position
        sphere.center += norm(pt_diff) * terrain_normal;

        // compute new speed
        vec3 v_normal = dot(sphere.speed, terrain_normal)*terrain_normal;
        vec3 v_tan = sphere.speed - v_normal;
        sphere.speed = 1.0f*v_tan - 0.0f*v_normal;

        // sedimentation
        if (sphere.falling_under_atm_rho == false) sphere.rho -= 0.001*frame_nb*t_step *norm(sphere.speed);
        else sphere.rho -= 0.000005*frame_nb*t_step * norm(sphere.speed);
        // find buffer for low density particles not in buffer:
        if (idx >=0 && sphere.rho < compute_atm_density(sphere.center.z))
        {
            sphere.falling_under_atm_rho = true;

            // add particle in a buffer

            // test proximity with existing buffers
            bool is_in_buffer = false;
            for (unsigned int j = 0; j<falling_spheres_buffers.size(); j++)
            {
                if (norm(falling_spheres_buffers[j][0].center - sphere.center) < 5*sphere.r)
                {
                    falling_spheres_buffers[j].push_back(sphere);
                    falling_spheres.erase(falling_spheres.begin()+idx);
                    is_in_buffer = true;
                    break;
                }
            }
            // if not cloase to any existing buffer, create a new one
            if (is_in_buffer == false)
            {
                std::vector<free_sphere_params> new_buffer;
                new_buffer.push_back(sphere);
                falling_spheres_buffers.push_back(new_buffer);
                falling_spheres.erase(falling_spheres.begin()+idx);
            }
        }
    }

}

void scene_model::ground_falling_sphere_update(free_sphere_params& sphere, int idx, unsigned int frame_nb)
{
    // find closest layer for radial force
    unsigned int closest_layer_id = 0;
    float min_dist = norm (sphere.center - smoke_layers[0].center);
    for (unsigned int j = 0; j<smoke_layers.size(); j++)
    {
        float dist = norm(sphere.center - smoke_layers[j].center);
        if (dist < min_dist && smoke_layers[j].rising && smoke_layers[j].secondary_plume == false)
        {
            min_dist = dist;
            closest_layer_id = j;
        }
    }

    // precomputation
    float V = 4./3. * 3.14 * sphere.r * sphere.r * sphere.r;
    float m = sphere.rho * V;

    float atm_rho = compute_atm_density(sphere.center.z);
    vec3 gravity = vec3(0,0,-m*g);
    vec3 buoyancy = vec3(0,0,atm_rho*V*g);
    vec3 friction = - 0.1 * normalize(sphere.speed);
    friction = vec3(0,0,0);

    // tests for additional force to prevent from being to close from column (not working)
//    vec3 column_interaction_force = vec3(0,0,0);
//    float terrain_z = field_height_at(sphere.center.x, sphere.center.y);
//    if (min_dist < 1.5*smoke_layers[closest_layer_id].r)
//    {
//        vec3 force_vector = normalize(sphere.center - smoke_layers[closest_layer_id].center);
//        force_vector = vec3(force_vector.x, force_vector.y, 0);
//        column_interaction_force = 0.1 * m * (2*smoke_layers[closest_layer_id].r - min_dist) * force_vector;
//    }
//    else if (min_dist < 2.5*smoke_layers[closest_layer_id].r && terrain_z + 100. < sphere.center.z)
//    {
//        // horizontal friction
//        column_interaction_force = -0.001 * m/t_step * vec3(sphere.speed.x, sphere.speed.y, 0);
//    }
//    if (smoke_layers[closest_layer_id].secondary_plume) column_interaction_force = vec3(0,0,0);
//    column_interaction_force = vec3(0,0,0);

    vec3 forces = gravity + buoyancy + friction;
    vec3 a = forces/m;
    vec3 v = sphere.speed + a * frame_nb*t_step;
    vec3 p = sphere.center + v * frame_nb*t_step;

    sphere.speed = v;
    sphere.center = p;

    if (!sphere.falling_disappeared) sphere_ground_collision(sphere, idx, frame_nb);
}

void scene_model::secondary_columns_creation()
{
    // merge close buffers
    for (unsigned int i = 0; i<falling_spheres_buffers.size(); i++)
    {
        for (unsigned int j = i+1; j<falling_spheres_buffers.size(); j++)
        {
            if (norm(falling_spheres_buffers[i][0].center - falling_spheres_buffers[j][0].center) < 3*falling_spheres_buffers[i][0].r)
            {
                for (unsigned int k = 0; k<falling_spheres_buffers[j].size(); k++)
                {
                    falling_spheres_buffers[i].push_back(falling_spheres_buffers[j][k]);
                }
                falling_spheres_buffers.erase(falling_spheres_buffers.begin()+j);
            }
        }
    }

    // find big enough buffers
    for (unsigned int i = 0; i<falling_spheres_buffers.size(); i++)
    {
        float wanted_ray = falling_spheres_buffers[i][0].r;
        float wanted_volume = wanted_ray * 3.14 * wanted_ray*wanted_ray;
        float sphere_volume = 4./3.*3.14*falling_spheres_buffers[i][0].r*falling_spheres_buffers[i][0].r*falling_spheres_buffers[i][0].r;
        float nb_spheres_needed = wanted_volume/sphere_volume;
        nb_spheres_needed = 6;

        //find closest layer
        vec3 center_i = falling_spheres_buffers[i][0].center;
        int closest_layer_id = smoke_layers.size()-1;
        float min_dist = norm(center_i - smoke_layers[closest_layer_id].center);
        for (unsigned int j = 0; j<smoke_layers.size(); j++)
        {
            float dist = norm (center_i - smoke_layers[j].center);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_layer_id = j;
            }
        }

        if (falling_spheres_buffers[i].size() > nb_spheres_needed*3 && min_dist > wanted_ray)
        {
            // emit layer
            add_smoke_layer(5, falling_spheres_buffers[i][0].rho, wanted_ray*0.75, falling_spheres_buffers[i][0].center, true);
            add_free_spheres_for_one_layer(smoke_layers.size()-1);
            total_layers_ejected++;
            //if (debug_mode) std::cout << "SECONDARY LAYER ADDED OK" << std::endl;

            // remove corresponding particles
            for (unsigned int j = 0; j<nb_spheres_needed; j++)
            {
                falling_spheres.push_back(falling_spheres_buffers[i][0]);
                falling_spheres[falling_spheres.size()-1].falling_disappeared = true;
                falling_spheres[falling_spheres.size()-1].rho = 10.;
                falling_spheres_buffers[i].erase(falling_spheres_buffers[i].begin());
            }
        }
    }

    // test to check closest spheres every timestep without buffers: too slow
//    for (unsigned int i = 0; i<falling_spheres.size(); i++)
//    {
//        if (falling_spheres[i].falling_under_atm_rho)
//        {
//            std::vector<unsigned int> close_particle_light;
//            for (unsigned int j = 0; j<falling_spheres.size(); j++)
//            {
//                if (falling_spheres[j].falling_under_atm_rho && norm(falling_spheres[i].center - falling_spheres[j].center)< 2*falling_spheres[i].r)
//                {
//                    close_particle_light.push_back(j);
//                }
//            }

//            vec3 center_i = falling_spheres[i].center;
//            //find closest layer
//            int closest_layer_id = smoke_layers.size()-1;
//            float min_dist = norm(center_i - smoke_layers[closest_layer_id].center);
//            for (unsigned int j = 0; j<smoke_layers.size(); j++)
//            {
//                float dist = norm (center_i - smoke_layers[j].center);
//                if (dist < min_dist)
//                {
//                    min_dist = dist;
//                    closest_layer_id = j;
//                }
//            }

//            if (close_particle_light.size() > 6 && min_dist > falling_spheres[i].r)
//            {
//                // emit layer
//                add_smoke_layer(5, falling_spheres[i].rho, falling_spheres[i].r, falling_spheres[i].center, true);
//                add_free_spheres_for_one_layer(smoke_layers.size()-1);
//                total_layers_ejected++;

//                // remove corresponding particles
//                for (unsigned int j = 0; j<6; j++)
//                {
//                    falling_spheres.erase(falling_spheres.begin() + close_particle_light[j]);
//                }
//            }
//        }
//    }

}

void scene_model::falling_spheres_update(unsigned int frame_nb)
{
    // edit spheres, attached or not to a buffer
    for (int i = falling_spheres.size()-1; i>=0; i--)
    {
        ground_falling_sphere_update(falling_spheres[i], i, frame_nb);
    }
    for (unsigned int i = 0; i<falling_spheres_buffers.size(); i++)
    {
        for (unsigned int j = 0; j<falling_spheres_buffers[i].size(); j++)
        {
            ground_falling_sphere_update(falling_spheres_buffers[i][j], -1, frame_nb);
        }
    }

    for (int i = falling_spheres.size()-1; i>=0; i--)
    {
        if (falling_spheres[i].falling_disappeared && falling_spheres[i].center.z < -2000.)
        {
            falling_spheres.erase(falling_spheres.begin()+i);
        }
    }

    // emit new layers from buffers
    secondary_columns_creation();
}


//------------------------------------------------------------
//---------------------- FREE SPHERES ------------------------
//------------------------------------------------------------ */

void scene_model::add_free_sphere(unsigned int i, float angle, float size_fac)
{
    free_sphere_params sphere(smoke_layers[i].center, angle, size_fac * smoke_layers[i].r, smoke_layers[i].v.z);
    sphere.size_factor = size_fac;
    sphere.rho = smoke_layers[i].rho;
    float volume = 4.0/3.0 * 3.14 * sphere.r*sphere.r*sphere.r;
    sphere.mass = sphere.rho/volume;
    sphere.closest_layer_idx = i;

    if (smoke_layers[i].secondary_plume)
    {
        sphere.secondary_column = true;
        sphere.closest_layer_idx = i;
    }

    for (unsigned int i = 0; i<subspheres_number; i++)
    {
        // random angles
        float theta = 3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float phi = 2*3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

        subsphere_params subs = subsphere_params();
        subs.parent_id = free_spheres.size();
        subs.relative_position = vec3(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
        subs.center = sphere.center + sphere.r * subs.relative_position;
        //subs.size_ratio = 0.10 + 0.25 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        subs.size_ratio = 0.2;
        subs.r = sphere.r*subs.size_ratio;

        for (unsigned int j = 0; j<subsubspheres_number; j++)
        {
            float theta2 = 3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            float phi2 = 2*3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            subsphere_params subsubs = subsphere_params();
            subsubs.parent_id = s2_spheres.size();
            subsubs.relative_position = vec3(sin(theta2)*cos(phi2), sin(theta2)*sin(phi2), cos(theta2));
            subsubs.center = subs.center + subs.r * subsubs.relative_position;
            subsubs.r = subs.r/5.;
            s3_spheres.push_back(subsubs);
        }
        s2_spheres.push_back(subs);
    }
    free_spheres.push_back(sphere);
}

void scene_model::add_free_spheres_for_one_layer(unsigned int i)
{
    unsigned int nb_spheres = 6;
    float angle_offset = 2*3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    //determine size differences
    buffer<float> sizes;
    for (unsigned int k = 0; k<nb_spheres; k++)
    {
        sizes.push_back(0.5f + static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
    }
    float total = sizes[0] + sizes[1] + sizes[2] + sizes[3] + sizes[4] + sizes[5];
    float factor = (float)nb_spheres / total;
    for (unsigned int k = 0; k<sizes.size(); k++)
    {
        sizes[k] *= factor;
    }

    // add spheres
    for (unsigned int j = 0; j<nb_spheres; j++)
    {
        float angle = (float)j*2.0*3.14/6.0;
        add_free_sphere(i, angle+angle_offset, sizes[j]);
    }
}

float compute_gaussian_speed_in_layer(float v_z, float max_r, float r)
{
    float A = max_r/(2.*sqrt(log(2)));
    return 2. * v_z * exp(-r*r/(A*A));
}

void scene_model::subdivide_and_make_falling(unsigned int i)
{
    // update subspheres
    update_subspheres_params();

    // define ray of falling spheres; density same as free sphere
    float falling_ray = free_spheres[i].r/5.;
    float falling_volume = 4./3.*3.14*falling_ray*falling_ray*falling_ray;
    float free_sphere_volume = 4./3.*3.14*free_spheres[i].r*free_spheres[i].r*free_spheres[i].r;
    float n_float = free_sphere_volume/falling_volume;
    int n = (int)n_float;

    // start adding subspheres as falling spheres
    for (unsigned int j = 0; j<s2_spheres.size(); j++)
    {
        if (s2_spheres[j].parent_id == i)
        {
            free_sphere_params sphere = free_sphere_params(s2_spheres[j].center, s2_spheres[j].r, free_spheres[i].rho);
            sphere.falling = true;
            sphere.stagnate = false;
            falling_spheres.push_back(sphere);
            if (n>=0) n--;
        }
    }

    // add all falling spheres
    if (n>0)
    {
        for (unsigned int j = 0; j<n; j++)
        {
            float rand_r = free_spheres[i].r * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            float rand_phi = 3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            float rand_theta = 2*3.14 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            vec3 rand_vec(sin(rand_theta)*cos(rand_phi), sin(rand_theta)*sin(rand_phi), cos(rand_theta));
            vec3 new_center = free_spheres[i].center + rand_r * rand_vec; //random position inside free sphere
            free_sphere_params sphere = free_sphere_params(new_center, falling_ray, free_spheres[i].rho);
            falling_spheres.push_back(sphere);
        }
    }

    // make free sphere falling
    free_spheres[i].falling = true;
    if (debug_mode) std::cout << i << " falls" << std::endl;
}

void scene_model::update_free_spheres()
{
    for (int i = free_spheres.size()-1; i>=0; i--)
    {
        free_sphere_params& sphere_i = free_spheres[i];

        if (!sphere_i.stagnate_long && !sphere_i.falling)
        {
            // identify closest layer which is rising or begins falling (later : all layers in which the sphere is)
            int closest_layer_id = smoke_layers.size()-1;
            float min_dist = norm(sphere_i.center - smoke_layers[closest_layer_id].center);
            for (unsigned int j = 0; j<smoke_layers.size(); j++)
            {
                //float dist = abs(free_spheres[i].center.z - smoke_layers[j].center.z);
                float dist = norm (sphere_i.center - smoke_layers[j].center);
                if (dist < min_dist && (smoke_layers[j].rising || smoke_layers[j].begin_falling) && !smoke_layers[j].stagnates_long)
                {
                    min_dist = dist;
                    closest_layer_id = j;
                }
            }
            if (sphere_i.secondary_column) closest_layer_id = sphere_i.closest_layer_idx;
            if (sphere_i.stagnate || sphere_i.stagnate_long) closest_layer_id = sphere_i.closest_layer_idx;
            closest_layer_id = sphere_i.closest_layer_idx;

            // check if closest layer begins falling (if so, make sphere falling)
            if (smoke_layers[closest_layer_id].begin_falling)
            {
                subdivide_and_make_falling(i);
            }
            else if (smoke_layers[closest_layer_id].stagnates && !smoke_layers[closest_layer_id].stagnates_long && !sphere_i.stagnate)
            {
                // check if densities have become equal: if so, keep altitude in memory
                sphere_i.stagnate = true;
                sphere_i.closest_layer_idx = closest_layer_id;
                sphere_i.stagnation_altitude = sphere_i.center.z;
            }
            else if (sphere_i.stagnate && !sphere_i.stagnate_long && smoke_layers[sphere_i.closest_layer_idx].stagnates_long)
            {
                // check if closest layer has reached max altitude: if so, keep sphere position in memory for stagnation spreading function
                sphere_i.stagnate_long = true;
                sphere_i.max_altitude = sphere_i.center.z;
                sphere_i.center_at_max_altitude = sphere_i.center;
                sphere_i.layer_center_at_max_altitude = smoke_layers[closest_layer_id].center;
                sphere_i.xy_at_max_altitude = sqrt(sphere_i.center.x*sphere_i.center.x + sphere_i.center.y*sphere_i.center.y);
            }
            else
            {
                // get axial and radial composants relative to layer center
                vec3 p_relative = sphere_i.center - smoke_layers[closest_layer_id].center;
                vec3 p_axial = dot(p_relative, smoke_layers[closest_layer_id].plume_axis) * smoke_layers[closest_layer_id].plume_axis;
                vec3 p_radial = p_relative - p_axial;

                // update rotation axis with new radial vector (can change upon time because axis changes)
                sphere_i.angle_vector = p_radial/norm(p_radial);
                //sphere_i.rotation_axis = normalize(cross(smoke_layers[closest_layer_id].plume_axis, p_radial));

                // update radial position if too close from plume axis
                //if (norm(p_radial) < smoke_layers[closest_layer_id].r)*/ free_spheres[i].center = smoke_layers[closest_layer_id].center + p_axial + smoke_layers[closest_layer_id].r * normalize(p_radial);

                // update size according to layer
                // size of layer + perturbation (some spheres should grow much more than others, to create diversity)
                float new_r = sphere_i.size_factor * smoke_layers[closest_layer_id].r;

                // update radial speed and position by adding perturbation (one part is random and one depends on radial position, so that spheres do not stay in the middle of the plume and do not go away)
                float random_f = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                sphere_i.perturbation += new_r * 0.001 * 2.0 * (random_f - 0.5);
                if (sphere_i.perturbation > 100.) sphere_i.perturbation = 100.;
                if (sphere_i.perturbation < -100.) sphere_i.perturbation = -100.;
                float new_speed = compute_gaussian_speed_in_layer(smoke_layers[closest_layer_id].speed_along_axis, 2.*smoke_layers[closest_layer_id].r, norm(p_relative)); // axial speed
                if (smoke_layers[closest_layer_id].stagnates) new_speed = smoke_layers[closest_layer_id].speed_along_axis;
                float radial_speed = (smoke_layers[closest_layer_id].r - norm(p_radial))/0.5;
                if (smoke_layers[closest_layer_id].stagnates) radial_speed = 0;
                sphere_i.perturbation += radial_speed;
                //if (smoke_layers[closest_layer_id].stagnates) std::cout << sphere_i.perturbation << std::endl;
                if (smoke_layers[closest_layer_id].stagnates && sphere_i.perturbation < 0) sphere_i.perturbation = 0;
                radial_speed = 0;
                //std::cout << norm(p_relative) - smoke_layers[closest_layer_id].r << " " << free_spheres[i].perturbation << " " << radial_speed << std::endl;

                // update rotation speed with new speed and ray
                float new_angular_speed = new_speed/new_r;

                // update
                sphere_i.speed = new_speed * smoke_layers[closest_layer_id].plume_axis + (sphere_i.perturbation) * sphere_i.angle_vector;
                sphere_i.r = new_r;
                sphere_i.relative_distance = norm(sphere_i.center-smoke_layers[closest_layer_id].center);
                sphere_i.rho = smoke_layers[closest_layer_id].rho;

                if (!sphere_i.stagnate && smoke_layers[closest_layer_id].theta >1)
                {
                    sphere_i.angular_speed = new_angular_speed;
                    float new_angle = sphere_i.current_angle + sphere_i.angular_speed * t_step;
                    sphere_i.current_angle = new_angle;
                }
                else sphere_i.angular_speed = 0;
            }
        }
    }

    // update positions
    for (unsigned int i = 0; i<free_spheres.size(); i++)
    {
        if (!free_spheres[i].stagnate_long && !free_spheres[i].falling)
        {
            free_spheres[i].center += t_step * free_spheres[i].speed;
        }
        else if (free_spheres[i].falling)
        {
            free_spheres[i].center.z = smoke_layers[free_spheres[i].closest_layer_idx].center.z;
        }
        //update_spheres_on_free_sphere(i);
    }

    // make begin_falling layers falling
    for (int i = smoke_layers.size()-1; i>=0; i--)
    {
        if (smoke_layers[i].begin_falling && smoke_layers.size() >= 1)
        {
            smoke_layers[i].falling = true;
            smoke_layers[i].begin_falling = false;
            smoke_layers[i].rising = false;
            smoke_layers[i].plume = false;
            //smoke_layers[i].center.z = -2000;
        }
    }
}


//------------------------------------------------------------
//---------------------- STAGNATION --------------------------
//------------------------------------------------------------ */

void scene_model::update_stagnation_spheres()
{
    for (int i = free_spheres.size()-1; i>=0; i--)
    {
        if (free_spheres[i].stagnate_long)
        {
            // closest layer
            unsigned int closest_layer_id = free_spheres[i].closest_layer_idx;

            // get wind
            vec3 wind = compute_wind_vector(free_spheres[i].center.z);

            // get radial composant relative to layer center
            vec3 p_radial = vec3(free_spheres[i].center.x, free_spheres[i].center.y,0);

            // update rotation axis with new radial vector (can change upon time because axis changes)
            free_spheres[i].angle_vector = normalize(p_radial);

            // update radial speed and position by adding perturbation
            float speed_factor = norm(vec3(free_spheres[i].center.x, free_spheres[i].center.y,0))/2000.;
            free_spheres[i].perturbation = stagnation_speed/speed_factor;
            if (norm(wind) != 0)
            {
                speed_factor = norm(p_radial)/2000.;
                free_spheres[i].perturbation = stagnation_speed/speed_factor;
                if (dot(free_spheres[i].speed, p_radial) < 0) free_spheres[i].perturbation = 0;
            }

            //free_spheres[i].angle_vector = normalize(vec3(free_spheres[i].angle_vector.x, free_spheres[i].angle_vector.y, 0));

            // update
            free_spheres[i].speed = 0*free_spheres[i].speed + 1*(free_spheres[i].perturbation) * free_spheres[i].angle_vector;

            float dxy = norm(free_spheres[i].speed)*t_step;
            vec3 relative_pos_at_max_alt = free_spheres[i].center_at_max_altitude - free_spheres[i].layer_center_at_max_altitude;
            float relative_xy_at_max_alt = sqrt(relative_pos_at_max_alt.x*relative_pos_at_max_alt.x + relative_pos_at_max_alt.y*relative_pos_at_max_alt.y);
            float a = (free_spheres[i].max_altitude - free_spheres[i].stagnation_altitude) * relative_xy_at_max_alt;

            vec3 relative_pos = free_spheres[i].center - free_spheres[i].layer_center_at_max_altitude;
            float relative_xy = sqrt(relative_pos.x*relative_pos.x + relative_pos.y*relative_pos.y);
            float dz = -a * dxy / ((relative_xy)*(relative_xy));
            if (relative_xy < relative_xy_at_max_alt) dz = 0;
            float new_z = free_spheres[i].stagnation_altitude + a / (relative_xy);
            if (relative_xy <= relative_xy_at_max_alt) new_z = free_spheres[i].max_altitude;

            free_spheres[i].center.z = new_z;
            free_spheres[i].angular_speed = 0;
            free_spheres[i].rho = smoke_layers[closest_layer_id].rho;

            free_spheres[i].speed += wind;
        }
    }

    // update positions
    for (unsigned int i = 0; i<free_spheres.size(); i++)
    {
        if (free_spheres[i].stagnate_long)
        {
            free_spheres[i].center += t_step * free_spheres[i].speed;
            //update_spheres_on_stagnation_sphere(i);
            if (free_spheres[i].closest_layer_idx != -1)
            {
                vec3 relative_position = free_spheres[i].center - smoke_layers[free_spheres[i].closest_layer_idx].center;
                free_spheres[i].relative_distance = norm(relative_position);
                free_spheres[i].angle_vector = normalize(relative_position);
            }
        }
    }
}


//------------------------------------------------------------
//------------------------- EXPORT ---------------------------
//------------------------------------------------------------ */

void scene_model::update_subspheres_params()
{
    // normal spheres
    for (size_t i = 0; i<s2_spheres.size(); i++)
    {
        // apply parent transfo : s_i = s/5, t_i = s*R*offset_i  + t
        unsigned int idx_parent = s2_spheres[i].parent_id;

        try
        {
            if (!free_spheres[idx_parent].falling)
            {
                mat3 const R_parent = rotation_from_axis_angle_mat3(free_spheres[idx_parent].rotation_axis, free_spheres[idx_parent].current_angle);
                float r_parent = free_spheres[idx_parent].r;
                vec3 t_parent = free_spheres[idx_parent].center;

                s2_spheres[i].r = r_parent * s2_spheres[i].size_ratio;
                s2_spheres[i].center = r_parent * R_parent * s2_spheres[i].relative_position + t_parent;
            }
        }
        catch(std::exception& e)
        {
        	std::cout << "Error in update_subspheres_params: " << e.what() << std::endl;
		}
        
    }

    for (size_t j = 0; j<s3_spheres.size(); j++)
    {
        // apply parent transfo : sj = si/5, tj = si*offsetj  + ti
        unsigned int idx_parent = s3_spheres[j].parent_id;
        if (!free_spheres[s2_spheres[idx_parent].parent_id].falling)
        {
            float r_parent = s2_spheres[idx_parent].r;
            vec3 t_parent = s2_spheres[idx_parent].center;

            s3_spheres[j].r = r_parent/5.;
            s3_spheres[j].center = r_parent*s3_spheres[j].relative_position + t_parent;
        }
    }
}

void scene_model::export_spheres()
{
    update_subspheres_params();

    unsigned int frame_nb = frame_count/50;

    //std::string s1_spheres_file;
    //std::string s1s2_spheres_file;

    std::string spheres_centers_file;
    std::string densities_file;
    if (frame_nb < 10)
    {
        spheres_centers_file = "../output/spheres_00" + std::to_string(frame_nb) + ".txt";
        densities_file = "../output/densities_00" + std::to_string(frame_nb) + ".txt";
        //s1_spheres_file = "../output/s1_spheres_00" + std::to_string(frame_nb) + ".txt";
        //s1s2_spheres_file = "../output/s1s2_spheres_00" + std::to_string(frame_nb) + ".txt";
    }
    else if (frame_nb < 100)
    {
        spheres_centers_file = "../output/spheres_0" + std::to_string(frame_nb) + ".txt";
        densities_file = "../output/densities_0" + std::to_string(frame_nb) + ".txt";
        //s1_spheres_file = "../output/s1_spheres_0" + std::to_string(frame_nb) + ".txt";
        //s1s2_spheres_file = "../output/s1s2_spheres_0" + std::to_string(frame_nb) + ".txt";
    }
    else
    {
        spheres_centers_file = "../output/spheres_" + std::to_string(frame_nb) + ".txt";
        densities_file = "../output/densities_" + std::to_string(frame_nb) + ".txt";
        //s1_spheres_file = "../output/s1_spheres_" + std::to_string(frame_nb) + ".txt";
        //s1s2_spheres_file = "../output/s1s2_spheres_" + std::to_string(frame_nb) + ".txt";
    }
    std::ofstream spheres_centers = std::ofstream(spheres_centers_file.c_str());
    std::ofstream densities = std::ofstream(densities_file.c_str());

    //std::ofstream s1_spheres = std::ofstream(s1_spheres_file.c_str());
    //std::ofstream s1s2_spheres = std::ofstream(s1s2_spheres_file.c_str());

    // free + stagnation spheres NEW
    for (unsigned int i = 0; i<free_spheres.size(); i++)
    {
        if (true)
        {
            free_sphere_params s = free_spheres[i];
            if (spheres_centers) spheres_centers << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
            if (densities) densities << s.rho << std::endl;
            //if (s1_spheres) s1_spheres << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
            //if (s1s2_spheres) s1s2_spheres << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
        }
    }

    // subspheres NEW
    for (unsigned int i = 0; i<s2_spheres.size(); i++)
    {
        unsigned int parent_idx = s2_spheres[i].parent_id;
        if (!free_spheres[parent_idx].falling)
        {
            subsphere_params s = s2_spheres[i];
            if (spheres_centers) spheres_centers << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
            if (densities) densities << free_spheres[parent_idx].rho << std::endl;
            //if (s1s2_spheres) s1s2_spheres << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
        }
    }
    for (unsigned int i = 0; i<s3_spheres.size(); i++)
    {
        unsigned int parent_idx = s3_spheres[i].parent_id;
        if (!free_spheres[s2_spheres[parent_idx].parent_id].falling)
        {
            subsphere_params s = s3_spheres[i];
            if (spheres_centers) spheres_centers << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
            if (densities) densities << free_spheres[s2_spheres[parent_idx].parent_id].rho << std::endl;
        }
    }

    // falling spheres
    for (unsigned int i = 0; i<falling_spheres.size(); i++)
    {
        free_sphere_params s = falling_spheres[i];
        if (spheres_centers) spheres_centers << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
        if (densities) densities << s.rho << std::endl;

        //spheres on spheres
//        for (unsigned int j = 0; j<s.spheres_on_sphere.size(); j++)
//        {
//            sphere_on_free_spheres_params s_2 = s.spheres_on_sphere[j];
//            if (spheres_centers) spheres_centers << s_2.center.x/1000. << " " << s_2.center.z/1000. << " " << s_2.center.y/1000. << " " << s_2.r/1000. << std::endl;
//            if (densities) densities << s.rho << std::endl;

//            //spheres on spheres on spheres
//            for (unsigned int k = 0; k<s_2.spheres_on_sphere.size(); k++)
//            {
//                sphere_on_free_spheres_params s_3 = s_2.spheres_on_sphere[k];
//                if (spheres_centers) spheres_centers << s_3.center.x/1000. << " " << s_3.center.z/1000. << " " << s_3.center.y/1000. << " " << s_3.r/1000. << std::endl;
//                if (densities) densities << s.rho << std::endl;


//            }
//        }
    }

    // falling spheres buffers
    for (unsigned int k = 0; k<falling_spheres_buffers.size(); k++)
    {
        for (unsigned int i = 0; i<falling_spheres_buffers[k].size(); i++)
        {
            free_sphere_params s = falling_spheres_buffers[k][i];
            if (spheres_centers) spheres_centers << s.center.x/1000. << " " << s.center.z/1000. << " " << s.center.y/1000. << " " << s.r/1000. << std::endl;
            if (densities) densities << s.rho << std::endl;

            //spheres on spheres
//            for (unsigned int j = 0; j<s.spheres_on_sphere.size(); j++)
//            {
//                sphere_on_free_spheres_params s_2 = s.spheres_on_sphere[j];
//                if (spheres_centers) spheres_centers << s_2.center.x/1000. << " " << s_2.center.z/1000. << " " << s_2.center.y/1000. << " " << s_2.r/1000. << std::endl;
//                if (densities) densities << s.rho << std::endl;

//                //spheres on spheres on spheres
//                for (unsigned int k = 0; k<s_2.spheres_on_sphere.size(); k++)
//                {
//                    sphere_on_free_spheres_params s_3 = s_2.spheres_on_sphere[k];
//                    if (spheres_centers) spheres_centers << s_3.center.x/1000. << " " << s_3.center.z/1000. << " " << s_3.center.y/1000. << " " << s_3.r/1000. << std::endl;
//                    if (densities) densities << s.rho << std::endl;


//                }
//            }
        }
    }

}


//------------------------------------------------------------
//------------------------- SETUP ----------------------------
//------------------------------------------------------------ */

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    std::string base_path = "X:/GithubProjects/volcanicplume/code_plume/";

    // begin with timer stopped
    timer.stop();
    replay = false;
    frame_replay = 0;
    new_layer_delay = 0;
    total_layers_ejected = 0;
    nb_of_iterations = 0;
    frame_count = 0;
    export_data = false;
    gui_param.display_smoke_layers = true;
    gui_param.display_free_spheres = false;
    gui_param.display_subspheres = false;
    gui_param.display_spheres_with_subspheres = false;
    gui_param.display_billboards = false;
    debug_mode = true;
    float seed = time(0);
    srand(seed);
    if (seed_ofstream)
    {
        seed_ofstream.precision(10);
        seed_ofstream << seed;
    }

    gui.show_frame_camera = false; std::cout << "replay becomes false 0" << std::endl;

    // camera setup
    scene.camera.apply_rotation(0,0,0,1.2f);
    scene.camera.apply_scaling(100.0);
    scene.camera.apply_translation_in_screen_plane(0, -0.5);

    // Meshes setup
    layer_mesh = mesh_drawable( mesh_primitive_cylinder(0.1f, {0,0,0}, {0,0,0.01}));
    layer_mesh.shader = shaders["mesh"];
    layer_mesh.uniform.color = {0,0.5,1};

    mesh cyl = vcl::mesh_primitive_cylinder(2.5f, {0,0,1.5}, {0,0,-1.5}, 30, 30);
    mesh d1 = vcl::mesh_primitive_disc(2.5f, {0,0,1.5});
    mesh d2 = vcl::mesh_primitive_disc(2.5f, {0,0,-1.5});
    mesh t = cyl;t.push_back(d1); t.push_back(d2);

    generic_sphere_mesh = vcl::mesh_primitive_sphere();
    generic_sphere_mesh.texture_id = scene.texture_white;
    //generic_torus_mesh = vcl::mesh_primitive_torus(1.,1.,{0,0,0}, {0,0,-1});
    generic_torus_mesh = t;
    generic_torus_mesh.uniform.color = {1,0.5,0};
    generic_torus_mesh.shader = shaders["mesh"];
    generic_torus_mesh.texture_id = scene.texture_white;
    generic_torus_mesh.uniform.color_alpha = 0.6f;
    texture_smoke_id = create_texture_gpu( image_load_png(base_path + "/scenes/sources/smoke/images/texture_panache.png") );

    sphere = mesh_drawable( mesh_primitive_sphere(0.1f));
    sphere.shader = shaders["mesh"];
    sphere.uniform.color = {0,0.5,1};
    sphere.texture_id = scene.texture_white;

    subspheres = vcl::mesh_primitive_sphere(1.0, {0,0,0}, 10 ,20);
    subspheres.texture_id = scene.texture_white;
    subspheres.uniform.color = {0.6,0.5,0.5};
    subspheres.uniform.shading.diffuse = 0.8f;
    subspheres.uniform.shading.specular = 0.0f;

    GLuint const texture_billboard = create_texture_gpu(image_load_png(base_path + "/scenes/sources/smoke/terrains/smoke2.png"));
    quad = mesh_drawable(mesh_primitive_quad({-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}));
    quad.texture_id = texture_billboard;
    quad.uniform.shading.ambiant = 1.0;
    quad.uniform.shading.diffuse = 0.0;
    quad.uniform.shading.specular = 0.0;

    auto circle = vcl::curve_primitve_circle(30, 1.0, {0,0,0}, {0,0,1});
    sphere_circle = curve_drawable(circle);
    sphere_circle.shader = shaders["curve"];
    sphere_circle.uniform.color = {1,0,0};

    //sampling subpheres
    {
        int N = 60;
        for (int k = 0; k < N; ++k)
        {
            //uniform sampling on sphere
            float theta = 2*3.14f*vcl::rand_interval();
            float phi   = std::acos(1-2.0f*vcl::rand_interval());


            float x = std::sin(phi)*std::cos(theta);
            float y = std::sin(phi)*std::sin(theta);
            float z = std::cos(phi);

            vec3 p = {x,y,z};
            bool add = true;
            for (int k2 = 0; add==true && k2 < k; ++k2)
                if(norm(p-samples_subspheres[k2])<0.18f)
                    add=false;
            samples_subspheres.push_back({x,y,z});
        }
    }

    {
        mesh m0 = vcl::mesh_primitive_sphere(1.0, {0,0,0}, 5 ,5);
        mesh m1 = vcl::mesh_primitive_sphere(1.0, {0,0,0}, 8 ,8);
        mesh m2 = vcl::mesh_primitive_sphere(1.0, {0,0,0}, 10 , 10);

        int N = 60;
        for (int k = 0; k < N; ++k)
        {
            //uniform sampling on sphere
            float theta = 2*3.14f*vcl::rand_interval();
            float phi   = std::acos(1-2.0f*vcl::rand_interval());
            float r = vcl::rand_interval(0.8f,1.0f);

            float x = r*std::sin(phi)*std::cos(theta);
            float y = r*std::sin(phi)*std::sin(theta);
            float z = r*std::cos(phi);

            vec3 p = {x,y,z};
            bool add = true;
            for (int k2 = 0; add==true && k2 < k; ++k2)
                if(norm(p-samples_subspheres[k2])<0.18f)
                    add=false;
            samples_subspheres.push_back({x,y,z});
        }

        mesh m;
        m.push_back(m0);
        for (int sub = 0; sub < samples_subspheres.size(); ++sub) {
            mesh temp = m1;
            float r = vcl::rand_interval(0.18f,0.2f);

            // subspheres
            for (int k = 0; k < temp.position.size(); ++k)
            {
                vec3 p = r*temp.position[k] + samples_subspheres[sub];

                vec3 n0 = temp.normal[k];
                vec3 n1 = normalize(p);

                float d = norm(p);
                float alpha = 0.0;
                if(d>1.0f && d<1.2f)
                    alpha = (d-1.0f)/0.2f;
                if(d>1.2f)
                    alpha = 1.0f;

                vec3 n = (1-alpha)*n1 + alpha*n0; // hack normals
                temp.normal[k] = n;
                temp.position[k] = p;
            }

            m.push_back(temp);
        }

        subspheres_display = mesh_drawable(m) ;

        subspheres_display.texture_id = scene.texture_white;
        subspheres_display.uniform.color = {0.6,0.6,0.55};
        subspheres_display.uniform.shading.ambiant = 0.7f;
        subspheres_display.uniform.shading.diffuse = 0.3f;
        subspheres_display.uniform.shading.specular = 0.0f;
    }


    // Terrain setup
    mesh mesh_terrain = mesh_load_file_obj(base_path + "/scenes/sources/smoke/terrains/sthelens_detailed_sub5.obj");
    terrain = mesh_drawable(mesh_terrain);
    terrain.shader = shaders["mesh"];
    terrain.uniform.color = {0.36f, 0.32f, 0.24f};
    terrain.uniform.shading.specular = 0.0f;
    terrain.uniform.transform.rotation = rotation_from_axis_angle_mat3({1.0f,0,0}, 3.14f/2.0f);
    terrain.uniform.transform.scaling = 100.f;
    terrain.uniform.transform.translation = {37.,68.,-25.f};
    fill_height_field(mesh_terrain.position, mesh_terrain.normal, terrain);

    terrain_display = terrain;
    terrain_display.uniform.transform.scaling = 1./1.;
    terrain_display.texture_id = create_texture_gpu(image_load_png(base_path + "/scenes/sources/smoke/terrains/terrain.png"));
    terrain_display.uniform.color = {1,1,1};

    // Params setup
    is_wind = false;
    linear_wind_base = 15.;
    for(unsigned int i = 0; i<6; i++)
    {
        wind_altitudes.push_back(i*4000);
        winds.push_back(wind_structure(0,0));
    }

    stagnation_speed = 50;

    // coeff init
    air_incorporation_coeff = 5.;

    subspheres_number = 200;
    subsubspheres_number = 0;

    // Parameters : to be chosen by user
    T_0 = 1273.; // initial temp (K)
    theta_0 = 0.; // initial angle (rad)
    U_0 = 150.; // initial speed (m.s-1)
    n_0 = 0.03; // initial gas mass fraction
    z_0 = 0.; // initial altitude (m)
    r_0 = 100.; // initial radius (m)
    rho_0 = 200.;

    // Parameters : constants
    g = 9.81; // (m.s-2)
}


void scene_model::display(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    draw(terrain_display, scene.camera, shaders["mesh"]);
    //draw(terrain, scene.camera, shaders["wireframe"]);

    float ratio = 100.0;

    // Display torus
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (unsigned int i=0; i<smoke_layers.size(); i++)
    {
        smoke_layer lay = smoke_layers[i];

        generic_torus_mesh.uniform.transform.scaling = lay.r/ratio;
        generic_torus_mesh.uniform.transform.translation = vec3(lay.center.x/ratio, lay.center.y/ratio, lay.center.z/ratio);
        generic_torus_mesh.uniform.transform.rotation = rotation_from_axis_angle_mat3(lay.theta_axis, lay.theta-3.14/2.0);
        if(gui_param.display_smoke_layers) draw(generic_torus_mesh, scene.camera);
    }

//    glBindTexture(GL_TEXTURE_2D, texture_smoke_id);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

    // billboards
    if(gui_param.display_billboards)
    {
        glDepthMask(false);
        for (unsigned int j = 0; j<free_spheres.size(); j++)
        {
            mat3 const R = rotation_from_axis_angle_mat3(free_spheres[j].rotation_axis, free_spheres[j].current_angle);
            float new_scaling = free_spheres[j].r/ratio;
            //if (j==0) std::cout << new_scaling << std::endl;
            vec3 new_translation = free_spheres[j].center/ratio;
            generic_sphere_mesh.uniform.transform.translation = new_translation;
            generic_sphere_mesh.uniform.transform.scaling = new_scaling;
            generic_sphere_mesh.uniform.transform.rotation = R;
            generic_sphere_mesh.uniform.color = {1,1,1};

            float var = vcl::perlin(j,2);

            quad.uniform.transform.rotation = rotation_from_axis_angle_mat3(scene.camera.orientation.col(2), free_spheres[j].current_angle * dot(free_spheres[j].rotation_axis,scene.camera.orientation.col(2)) * 1.5f *(1+0.3*var)   + 2.2145*j*j) * scene.camera.orientation;
            quad.uniform.transform.translation = new_translation;
            quad.uniform.transform.scaling = new_scaling*1.3;
            quad.uniform.color_alpha = 0.8+0.3f*(2*var-1.0f);
            draw(quad, scene.camera, shaders["mesh"]);
        }
        glDepthMask(true);
    }


    // free + stagnation spheres display
    if(gui_param.display_free_spheres)
    {
        for (unsigned int j = 0; j<free_spheres.size(); j++)
        {
            //if (!free_spheres[j].falling)
            if(true)
            {
                mat3 const R = rotation_from_axis_angle_mat3(free_spheres[j].rotation_axis, free_spheres[j].current_angle);
                float r = free_spheres[j].r/ratio;
                vec3 t = free_spheres[j].center/ratio;
                float rho = free_spheres[j].rho;
                float disp_rho = 1. - rho;
                if (disp_rho < 0) disp_rho = 0.;
                disp_rho = 1.;

                generic_sphere_mesh.uniform.transform.translation = t;
                generic_sphere_mesh.uniform.transform.scaling = r;
                generic_sphere_mesh.uniform.transform.rotation = R;
                generic_sphere_mesh.uniform.color = {disp_rho,disp_rho,disp_rho};
                if(gui_param.display_free_spheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
            }
        }
    }

    // spheres+subspheres display (lighter)
    if(gui_param.display_spheres_with_subspheres)
    {
        for (unsigned int j = 0; j<free_spheres.size(); j++)
        {
            //if (!free_spheres[j].falling)
            if (true)
            {
                mat3 const R = rotation_from_axis_angle_mat3(free_spheres[j].rotation_axis, free_spheres[j].current_angle);
                float r = free_spheres[j].r/ratio;
                vec3 t = free_spheres[j].center/ratio;
                float rho = free_spheres[j].rho;
                float disp_rho = 1. - rho;
                if (disp_rho < 0) disp_rho = 0.;
                disp_rho = 1.;

                subspheres_display.uniform.transform.translation = t;
                subspheres_display.uniform.transform.scaling = r;
                subspheres_display.uniform.transform.rotation = R;
                subspheres_display.uniform.color = {disp_rho,disp_rho,disp_rho};

                draw(subspheres_display, scene.camera, shaders["mesh"]);
            }
        }
    }

    // subspheres display
    if(gui_param.display_subspheres)
    {
        for (unsigned int j = 0; j<s2_spheres.size(); j++)
        {
            if (!free_spheres[s2_spheres[j].parent_id].falling)
            {
                float r = s2_spheres[j].r/ratio;
                vec3 t = s2_spheres[j].center/ratio;
                float rho = free_spheres[s2_spheres[j].parent_id].rho;
                float disp_rho = 1. - rho;
                if (disp_rho < 0) disp_rho = 0.;
                disp_rho = 1.;

                generic_sphere_mesh.uniform.transform.translation = t;
                generic_sphere_mesh.uniform.transform.scaling = r;
                generic_sphere_mesh.uniform.color = {disp_rho,disp_rho,disp_rho};
                if(gui_param.display_subspheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
            }
        }
    }

    // falling spheres display
    for (unsigned int j = 0; j<falling_spheres.size(); j++)
    {
        float new_scaling = falling_spheres[j].r/ratio;
        vec3 new_translation = {falling_spheres[j].center.x/ratio, falling_spheres[j].center.y/ratio, falling_spheres[j].center.z/ratio};
        generic_sphere_mesh.uniform.transform.translation = new_translation;
        generic_sphere_mesh.uniform.transform.scaling = new_scaling;
        generic_sphere_mesh.uniform.transform.rotation = mat3::identity();
        if (falling_spheres[j].falling_under_atm_rho) generic_sphere_mesh.uniform.color = {1,0,0};
        else generic_sphere_mesh.uniform.color = {1,1,1};
        if(gui_param.display_free_spheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
    }
    // buffer falling spheres display
    for (unsigned int k = 0; k<falling_spheres_buffers.size(); k++)
    {
        for (unsigned int j = 0; j<falling_spheres_buffers[k].size(); j++)
        {
            float new_scaling = falling_spheres_buffers[k][j].r/ratio;
            vec3 new_translation = {falling_spheres_buffers[k][j].center.x/ratio, falling_spheres_buffers[k][j].center.y/ratio, falling_spheres_buffers[k][j].center.z/ratio};
            generic_sphere_mesh.uniform.transform.translation = new_translation;
            generic_sphere_mesh.uniform.transform.scaling = new_scaling;
            generic_sphere_mesh.uniform.transform.rotation = mat3::identity();
            if (falling_spheres_buffers[k][j].falling_under_atm_rho) generic_sphere_mesh.uniform.color = {1,0,0};
            else generic_sphere_mesh.uniform.color = {1,1,1};
            if(gui_param.display_free_spheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
        }
    }

    glBindTexture(GL_TEXTURE_2D, scene.texture_white);

}

void scene_model::display_replay(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui, size_t frame)
{
    draw(terrain_display, scene.camera, shaders["mesh"]);

    float ratio = 100.0;

    // Display torus
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (unsigned int i=0; i<smoke_layers_frames[frame].size(); i++)
    {
        smoke_layer lay = smoke_layers_frames[frame][i];

        generic_torus_mesh.uniform.transform.scaling = lay.r/ratio;
        generic_torus_mesh.uniform.transform.translation = vec3(lay.center.x/ratio, lay.center.y/ratio, lay.center.z/ratio);
        generic_torus_mesh.uniform.transform.rotation = rotation_from_axis_angle_mat3(lay.theta_axis, lay.theta-3.14/2.0);
        if(gui_param.display_smoke_layers) draw(generic_torus_mesh, scene.camera);
    }


//    glBindTexture(GL_TEXTURE_2D, texture_smoke_id);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

    // billboards
    if(gui_param.display_billboards)
    {
        glDepthMask(false);
        for (unsigned int j = 0; j<free_spheres_frames[frame].size(); j++)
        {
            mat3 const R = rotation_from_axis_angle_mat3(free_spheres_frames[frame][j].rotation_axis, free_spheres_frames[frame][j].current_angle);
            float new_scaling = free_spheres_frames[frame][j].r/ratio;
            vec3 new_translation = free_spheres_frames[frame][j].center/ratio;
            generic_sphere_mesh.uniform.transform.translation = new_translation;
            generic_sphere_mesh.uniform.transform.scaling = new_scaling;
            generic_sphere_mesh.uniform.transform.rotation = R;
            generic_sphere_mesh.uniform.color = {1,1,1};

            float var = vcl::perlin(j,2);

            quad.uniform.transform.rotation = rotation_from_axis_angle_mat3(scene.camera.orientation.col(2), free_spheres_frames[frame][j].current_angle * dot(free_spheres_frames[frame][j].rotation_axis,scene.camera.orientation.col(2)) * 1.5f *(1+0.3*var)   + 2.2145*j*j) * scene.camera.orientation;
            quad.uniform.transform.translation = new_translation;
            quad.uniform.transform.scaling = new_scaling;
            quad.uniform.color_alpha = 0.8+0.3f*(2*var-1.0f);
            draw(quad, scene.camera, shaders["mesh"]);
        }
        glDepthMask(true);
    }

    // free + stagnation spheres display NEW
    for (unsigned int j = 0; j<free_spheres_frames[frame].size(); j++)
    {
        if (!free_spheres_frames[frame][j].falling)
        {
            mat3 const R = rotation_from_axis_angle_mat3(free_spheres_frames[frame][j].rotation_axis, free_spheres_frames[frame][j].current_angle);
            float r = free_spheres_frames[frame][j].r/ratio;
            vec3 t = free_spheres_frames[frame][j].center/ratio;
            float rho = free_spheres_frames[frame][j].rho;
            float disp_rho = 1. - rho;
            if (disp_rho < 0) disp_rho = 0.;

            generic_sphere_mesh.uniform.transform.translation = t;
            generic_sphere_mesh.uniform.transform.scaling = r;
            generic_sphere_mesh.uniform.transform.rotation = R;
            generic_sphere_mesh.uniform.color = {disp_rho,disp_rho,disp_rho};
            if(gui_param.display_free_spheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
        }
    }

    // spheres+subspheres display (lighter)
    if(gui_param.display_spheres_with_subspheres)
    {
        for (unsigned int j = 0; j<free_spheres_frames[frame].size(); j++)
        {
            if (!free_spheres_frames[frame][j].falling)
            {
                mat3 const R = rotation_from_axis_angle_mat3(free_spheres_frames[frame][j].rotation_axis, free_spheres_frames[frame][j].current_angle);
                float r = free_spheres_frames[frame][j].r/ratio;
                vec3 t = free_spheres_frames[frame][j].center/ratio;
                float rho = free_spheres_frames[frame][j].rho;
                float disp_rho = 1. - rho;
                if (disp_rho < 0) disp_rho = 0.;

                subspheres_display.uniform.transform.translation = t;
                subspheres_display.uniform.transform.scaling = r;
                subspheres_display.uniform.transform.rotation = R;
                subspheres_display.uniform.color = {disp_rho,disp_rho,disp_rho};

                draw(subspheres_display, scene.camera, shaders["mesh"]);
            }
        }
    }

    // subspheres display NEW
    if(gui_param.display_subspheres)
    {
        for (unsigned int j = 0; j<s2_spheres_frames[frame].size(); j++)
        {
            if (!free_spheres[s2_spheres_frames[frame][j].parent_id].falling)
            {
                float r = free_spheres_frames[frame][j].r/ratio;
                vec3 t = free_spheres_frames[frame][j].center/ratio;
                float rho = free_spheres_frames[frame][j].rho;
                float disp_rho = 1. - rho;
                if (disp_rho < 0) disp_rho = 0.;

                generic_sphere_mesh.uniform.transform.translation = t;
                generic_sphere_mesh.uniform.transform.scaling = r;
                generic_sphere_mesh.uniform.color = {disp_rho,disp_rho,disp_rho};
                if(gui_param.display_subspheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
            }
        }
    }

    // falling spheres display
    for (unsigned int j = 0; j<falling_spheres_frames[frame].size(); j++)
    {
        float new_scaling = falling_spheres_frames[frame][j].r/ratio;
        vec3 new_translation = {falling_spheres_frames[frame][j].center.x/ratio, falling_spheres_frames[frame][j].center.y/ratio, falling_spheres_frames[frame][j].center.z/ratio};
        generic_sphere_mesh.uniform.transform.translation = new_translation;
        generic_sphere_mesh.uniform.transform.scaling = new_scaling;
        generic_sphere_mesh.uniform.transform.rotation = mat3::identity();
        if (falling_spheres_frames[frame][j].falling_under_atm_rho) generic_sphere_mesh.uniform.color = {1,0,0};
        else generic_sphere_mesh.uniform.color = {1,1,1};
        if(gui_param.display_free_spheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
    }
    // buffer falling spheres display
    for (unsigned int k = 0; k<falling_spheres_buffers_frames[frame].size(); k++)
    {
        for (unsigned int j = 0; j<falling_spheres_buffers_frames[frame][k].size(); j++)
        {
            float new_scaling = falling_spheres_buffers_frames[frame][k][j].r/ratio;
            vec3 new_translation = {falling_spheres_buffers_frames[frame][k][j].center.x/ratio, falling_spheres_buffers_frames[frame][k][j].center.y/ratio, falling_spheres_buffers_frames[frame][k][j].center.z/ratio};
            generic_sphere_mesh.uniform.transform.translation = new_translation;
            generic_sphere_mesh.uniform.transform.scaling = new_scaling;
            generic_sphere_mesh.uniform.transform.rotation = mat3::identity();
            if (falling_spheres_buffers_frames[frame][k][j].falling_under_atm_rho) generic_sphere_mesh.uniform.color = {1,0,0};
            else generic_sphere_mesh.uniform.color = {1,1,1};
            if(gui_param.display_free_spheres) draw(generic_sphere_mesh, scene.camera, shaders["mesh"]);
        }
    }

    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
}


void scene_model::set_gui()
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    // Parameters

    ImGui::Checkbox("Display torus layers", &gui_param.display_smoke_layers);
    ImGui::Checkbox("Display free spheres", &gui_param.display_free_spheres);
    //ImGui::Checkbox("Display subspheres", &gui_param.display_subspheres);
    ImGui::Checkbox("Display spheres with subspheres", &gui_param.display_spheres_with_subspheres);
    ImGui::Checkbox("Display billboards", &gui_param.display_billboards);

    unsigned int spheres_min=0, spheres_max=500;
    ImGui::SliderScalar("Number of subspheres", ImGuiDataType_S32, &subspheres_number, &spheres_min, &spheres_max);
    ImGui::SliderScalar("Number of subsubspheres", ImGuiDataType_S32, &subsubspheres_number, &spheres_min, &spheres_max);

    // Coeffs

    //float air_inc_min = 0.5, air_inc_max = 10.;
    //ImGui::SliderScalar("Air incorporation coefficient", ImGuiDataType_Float, &air_incorporation_coeff, &air_inc_min, &air_inc_max, "%.2f");

    // Initial conditions

    float initial_speed_min = 0., initial_speed_max = 200.;
    ImGui::SliderScalar("Initial plume speed", ImGuiDataType_Float, &U_0, &initial_speed_min, &initial_speed_max, "%.2f m/s");
    float initial_density_min = 150., initial_density_max = 250.;
    ImGui::SliderScalar("Initial plume density", ImGuiDataType_Float, &rho_0, &initial_density_min, &initial_density_max, "%.2f kg/m3");
    float vent_ray_min = 50., vent_ray_max = 200.;
    ImGui::SliderScalar("Vent radius", ImGuiDataType_Float, &r_0, &vent_ray_min, &vent_ray_max, "%.2f m");
    float vent_altitude_min = 0., vent_altitude_max = 8000.;
    ImGui::SliderScalar("Vent altitude", ImGuiDataType_Float, &z_0, &vent_altitude_min, &vent_altitude_max, "%.2f m");


    // Wind presets
    if (ImGui::Button("No wind"))
    {
        is_wind = false;
        for(unsigned int i = 0; i<winds.size(); i++)
        {
            winds[i].intensity = 0;
            winds[i].angle = 0;
            winds[i].wind_vector = vec3(1,0,0);
        }
    }
    if (ImGui::Button("Linear Wind"))
    {
        is_wind = true;
        for(unsigned int i = 0; i<winds.size(); i++)
        {
            winds[i].intensity = i*linear_wind_base;
            if (i>3) winds[i].intensity = 3*linear_wind_base;
            if (winds[i].intensity == 0) winds[i].intensity = 1;
            winds[i].angle = 0;
            winds[i].wind_vector = winds[i].intensity * vec3(cos(winds[i].angle), sin(winds[i].angle),0);
        }
    }

    // Wind

    float lin_windbase_min = 0., lin_windbase_max = 35.;
    if (ImGui::SliderScalar("Linear wind base speed", ImGuiDataType_Float, &linear_wind_base, &lin_windbase_min, &lin_windbase_max, "%1.f m/s"))
    {
        if (is_wind)
        {
            for(unsigned int i = 0; i<winds.size(); i++)
            {
                winds[i].intensity = i*linear_wind_base;
                if (i>3) winds[i].intensity = 3*linear_wind_base;
                if (winds[i].intensity == 0) winds[i].intensity = 1;
                winds[i].angle = 0;
                winds[i].wind_vector = winds[i].intensity * vec3(cos(winds[i].angle), sin(winds[i].angle),0);
            }
        }
    }

    int wind_min = 0;
    int wind_max = 300;
    int angle_min = 0, angle_max = 360;
    for (int i = wind_altitudes.size()-1; i>=0; i--)
    {
        std::string alti = "Intensity (" + std::to_string(wind_altitudes[i]) + "m)";
        std::string angl = "Angle (" + std::to_string(wind_altitudes[i]) + "m)";
        if (ImGui::SliderScalar(alti.c_str(), ImGuiDataType_S32, &winds[i].intensity, &wind_min, &wind_max))
            winds[i].wind_vector = winds[i].intensity * vec3(cos(winds[i].angle), sin(winds[i].angle),0);
        if (ImGui::SliderScalar(angl.c_str(), ImGuiDataType_S32, &winds[i].angle, &angle_min, &angle_max))
            winds[i].wind_vector = winds[i].intensity * vec3(cos(winds[i].angle), sin(winds[i].angle),0);
    }

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();
    if (ImGui::Button("Stop and reset"))
    {
        timer.stop();
        smoke_layers.clear();
        free_spheres.clear();
        falling_spheres.clear();
        stagnate_spheres.clear();
        falling_spheres_buffers.clear();
        frame_count = 0;

        smoke_layers_frames.clear();
        free_spheres_frames.clear();
        s2_spheres_frames.clear();
        stagnate_spheres_frames.clear();
        falling_spheres_frames.clear();
        falling_spheres_buffers_frames.clear();
        frame_replay = 0;
        replay = false;
        export_data = false;
    }
    if (ImGui::Button("Replay"))
    {
        if (!export_data)
        {
            timer.stop();
            frame_replay = 0;
            replay = true; std::cout << "replay becomes true" << std::endl;
        }
    }
    if (ImGui::Button("Stop Replay"))
    {
        timer.start();
        replay = false; std::cout << "replay becomes false" << std::endl;
    }
    if (ImGui::Button("Export data during simulation (slow, no replay, activate before starting simulation)"))
    {
        export_data = true;
        replay = false;
    }

}

