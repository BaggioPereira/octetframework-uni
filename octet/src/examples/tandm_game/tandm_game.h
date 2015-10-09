////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//

#include <fstream>
#include <sstream>

namespace octet {
  /// Scene containing a box with octet.
  class tandm_game : public app {
    // scene for drawing box
    ref<visual_scene> app_scene;
	btDefaultCollisionConfiguration config;
	btCollisionDispatcher *dispatcher;
	btDbvtBroadphase *broadphase;
	btSequentialImpulseConstraintSolver *solver;
	btDiscreteDynamicsWorld *world;

	mat4t worldCoord;

	dynarray<btRigidBody*> rigid_bodies;
	mesh_box *box;
	mesh_sphere *sph;
	material *wall, *floor, *flip, *end, *player;

	scene_node *cam;

	string contents;

	int levelNumber = 1;

  public:
    /// this is called when we construct the class before everything is initialised.
    tandm_game(int argc, char **argv) : app(argc, argv) 
	{
		dispatcher = new btCollisionDispatcher(&config);
		broadphase = new btDbvtBroadphase();
		solver = new btSequentialImpulseConstraintSolver();
		world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, &config);
    }

	//destructor
	~tandm_game()
	{
		delete world;
		delete solver;
		delete broadphase;
		delete dispatcher;
	}

	//add a mesh with rigid body
	void add_part(mat4t_in coord, mesh *shape, material *mat, bool is_dynamic)
	{
		scene_node *node = new scene_node();
		node->access_nodeToParent() = coord;
		app_scene->add_child(node);
		app_scene->add_mesh_instance(new mesh_instance(node, shape, mat));

		btMatrix3x3 matrix(get_btMatrix3x3(coord));
		btVector3 pos(get_btVector3(coord[3].xyz()));

		btCollisionShape *collisionShape = shape->get_bullet_shape();
		if (collisionShape)
		{
			btTransform transform(matrix, pos);
			btDefaultMotionState *motionState = new btDefaultMotionState(transform);
			btScalar mass = is_dynamic ? 0.5f : 0.0f;
			btVector3 inertiaTensor;

			collisionShape->calculateLocalInertia(mass, inertiaTensor);

			btRigidBody *rigid_body = new btRigidBody(mass, motionState, collisionShape, inertiaTensor);
			world->addRigidBody(rigid_body);
			rigid_bodies.push_back(rigid_body);
			rigid_body->setUserPointer(node);
		}
	}

	//clear the scene
	void newScene()
	{
		app_scene->reset();
		app_scene->create_default_camera_and_lights();
		app_scene->get_camera_instance(0)->set_far_plane(1000);
		cam = app_scene->get_camera_instance(0)->get_node();
		cam->translate(vec3(24, -24, 50));
	}

	//read txt file and get level data
	void loadTxt()
	{
		std::fstream myFile;
		std::stringstream fileName;
		fileName << "blanklevel" << ".txt";
		myFile.open(fileName.str().c_str(), std::ios::in);
		if (!myFile.is_open())
		{
			printf("File not opened/is missing\n");
		}

		else
		{
			printf("File open\n");
			myFile.seekg(0, myFile.end);
			int length = myFile.tellg();
			myFile.seekg(0, myFile.beg);

			std::stringstream file;
			file.str(std::string());
			file << myFile.rdbuf();
			myFile.close();

			contents = file.str().c_str();
			//printf("%s\n", contents.c_str());
			createLevel();
		}
	}

	//create the level according to the txt file
	void createLevel()
	{
		vec3 pos = vec3(0, 0, 0);
		int x = 0;
		for (int i = 0; i < contents.size(); i++)
		{
			scene_node *node = new scene_node;
			char c = contents[i];
			switch (c)
			{
			case '\n': pos -= (vec3(x, 0, 0));
				x = 0;
				pos += vec3(0, -1, 0);
				break;
			case ' ': 
			case '/':
				x += 1; 
				pos += vec3(1, 0, 0);
				break;
			case '_': worldCoord.translate(pos);
				add_part(worldCoord, box, floor, false);
				rigid_bodies.back()->setFriction(0);
				rigid_bodies.back()->setRestitution(0);
				printf("This is floor %d\n", rigid_bodies.size());
				worldCoord.loadIdentity();
				/*node->translate(pos);
				app_scene->add_child(node);
				app_scene->add_mesh_instance(new mesh_instance(node, box, floor));*/
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'B':
			case 'F': worldCoord.translate(pos);
				add_part(worldCoord, box, flip, false);
				rigid_bodies.back()->setFriction(0);
				rigid_bodies.back()->setRestitution(0);
				printf("This is flip/blocks %d\n", rigid_bodies.size());
				worldCoord.loadIdentity();
				/*node->translate(pos);
				app_scene->add_child(node);
				app_scene->add_mesh_instance(new mesh_instance(node, box, flip));*/
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'E': worldCoord.translate(pos);
				add_part(worldCoord, box, end, false);
				rigid_bodies.back()->setFriction(0);
				rigid_bodies.back()->setRestitution(0);
				printf("This is end block %d\n", rigid_bodies.size());
				worldCoord.loadIdentity();
				/*node->translate(pos);
				app_scene->add_child(node);
				app_scene->add_mesh_instance(new mesh_instance(node, box, end));*/
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'P': worldCoord.translate(pos);
				add_part(worldCoord, box, player, true);
				rigid_bodies.back()->setFriction(0);
				rigid_bodies.back()->setRestitution(0);
				printf("This is player %d\n", rigid_bodies.size());
				worldCoord.loadIdentity();
				/*node->translate(pos);
				app_scene->add_child(node);
				app_scene->add_mesh_instance(new mesh_instance(node, box, player));*/
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case '-':
			case '¦': worldCoord.translate(pos);
				add_part(worldCoord, box, wall, false);
				rigid_bodies.back()->setFriction(0);
				rigid_bodies.back()->setRestitution(0);
				printf("This is wall %d\n", rigid_bodies.size());
				worldCoord.loadIdentity();
				/*node->translate(pos);
				app_scene->add_child(node);
				app_scene->add_mesh_instance(new mesh_instance(node, box, wall));*/
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			default : break;
			}
		}
	}

    /// this is called once OpenGL is initialized
    void app_init() {
      app_scene =  new visual_scene();
	  box = new mesh_box(0.5f);
	  sph = new mesh_sphere(vec3(0, 0, 0), 1, 1);
	  wall = new material(vec4(1, 0, 0, 1));
	  floor = new material(vec4(0, 1, 0, 1));
	  flip = new material(vec4(0, 0, 1, 1));
	  end = new material(vec4(1, 1, 1, 1));
	  player = new material(vec4(0, 1, 1, 0));
	  newScene();
	  loadTxt();
    }

    /// this is called to draw the world
    void draw_world(int x, int y, int w, int h) {
      int vx = 0, vy = 0;
      get_viewport_size(vx, vy);
      app_scene->begin_render(vx, vy);

	  world->stepSimulation(1.0f / 30, 1.0f / 30, 1.0f / 30);
	  btCollisionObjectArray &colArray = world->getCollisionObjectArray();
	  for (unsigned i = 0; i != colArray.size(); ++i)
	  {
		  btCollisionObject *colObj = colArray[i];
		  scene_node *node = (scene_node *)colObj->getUserPointer();
		  if (node)
		  {
			  mat4t &modelToWorld = node->access_nodeToParent();
			  colObj->getWorldTransform().getOpenGLMatrix(modelToWorld.get());
		  }
	  }

      // update matrices. assume 30 fps.
      app_scene->update(1.0f/30);

      // draw the scene
      app_scene->render((float)vx / vy);

	  if (is_key_going_down(key_up))
	  {
		  rigid_bodies[480]->applyCentralForce(btVector3(0, 100, 0));
	  }

	  else if (is_key_down(key_right))
	  {
		  rigid_bodies[480]->applyCentralForce(btVector3(10, 0, 0));
	  }

	  else if (is_key_down(key_left))
	  {
		  rigid_bodies[480]->applyCentralForce(btVector3(-10, 0, 0));
	  }
    }
  };
}
