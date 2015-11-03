////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//

#include <fstream>
#include <sstream>
#include <Xinput.h>

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
	btRigidBody *staticObject;
	dynarray<btRigidBody*> rigid_bodies;
	dynarray<btHingeConstraint*> flippers;
	dynarray<btGeneric6DofSpringConstraint*> springBodies;
	mesh_box *box, *flipperMesh, *blockerMesh;
	mesh_sphere *sph;
	material *wall, *floor, *flip, *end, *player;

	scene_node *cam;

	string contents;
	int player_node;

	//Xbox controller enums and state
	XINPUT_STATE state;
	enum BUTTONS {
		Left = 0, 
		Right, 
		Up, 
		Down, 
		Start, 
		Back
	};

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

	//call to create mesh and set position with friction and restitution
	void add_rigid_body(vec3 position, mesh *msh, material *mat, char letter, bool active)
	{
		worldCoord.translate(position);
		add_part(worldCoord, msh, mat, active);
		
		rigid_bodies.back()->setFriction(0);
		rigid_bodies.back()->setRestitution(0);

		//static object 
		if (letter == 'O')
		{
			staticObject = rigid_bodies.back();
		}
		
		//player object
		if (letter == 'P')
		{
			player_node = rigid_bodies.size() - 1; //gets the player node
			rigid_bodies.back()->setLinearFactor(btVector3(1, 1, 0)); //constraints z axis movement
			rigid_bodies.back()->setAngularFactor(btVector3(0, 0, 1)); //constraints x and y axis rotation
		}

		//spring blockers
		if (letter == 'B')
		{
			btTransform localA, localB;
			localA.setIdentity();
			localB.setIdentity();
			localA.getOrigin() = btVector3(position.x(), position.y(), position.z());
			btRigidBody *springBody = rigid_bodies.back();
			springBody->setLinearFactor(btVector3(0, 1, 0));
			btGeneric6DofSpringConstraint *springConstraint = new btGeneric6DofSpringConstraint(*staticObject, *springBody, localA, localB, true);
			springConstraint->setLimit(0, 0, 0); //X Axis
			springConstraint->setLimit(1, 3, -3); //Y Axis
			springConstraint->setLimit(2, 3, -3); //Z Axis
			springConstraint->setLimit(3, 0, 0); 
			springConstraint->setLimit(4, 0, 0); 
			springConstraint->setLimit(5, 0, 0); 
			springConstraint->enableSpring(1, true); //int index implies the axis you want to move in
			springConstraint->setStiffness(1, 100);
			world->addConstraint(springConstraint);
			rigid_bodies.back()->applyCentralForce(btVector3(0, 100, 0));
			springBodies.push_back(springConstraint);
		}


		//NEEDS WORK, FLIPPERS NOT BEING SET TO CORRECT PIVOT POINTS
		/*if (letter == 'F')
		{
			btVector3 offset = rigid_bodies.back()->getCenterOfMassPosition();
			offset = btVector3(-offset.x()*0.25f, offset.y(), offset.z());
			btHingeConstraint *flipperHinge =new btHingeConstraint(*rigid_bodies.back(), offset, btVector3(0, 0, 1), true);
			flipperHinge->setLimit(-3.14f*0.5f, 3.14f*0.0f);
			world->addConstraint(flipperHinge);
			flippers.push_back(flipperHinge);
		}*/

		worldCoord.loadIdentity();
	}

	//clear the scene
	void newScene()
	{
		rigid_bodies.reset();
		flippers.reset();
		app_scene->reset();
		app_scene->create_default_camera_and_lights();
		app_scene->get_camera_instance(0)->set_far_plane(1000);
		cam = app_scene->get_camera_instance(0)->get_node();
		cam->translate(vec3(24, -24, 50));
		box = new mesh_box(0.5f);
		flipperMesh = new mesh_box(vec3(0.5f, 0.25f, 0.5f));
		blockerMesh = new mesh_box(vec3(0.5f, 1, 0.5f));
		sph = new mesh_sphere(vec3(0, 0, 0), 1, 1);
		wall = new material(vec4(1, 0, 0, 1));
		floor = new material(vec4(0, 1, 0, 1));
		flip = new material(vec4(0, 0, 1, 1));
		end = new material(vec4(1, 1, 1, 1));
		player = new material(vec4(0, 1, 1, 0));

		//Xbox Controller detection code
		DWORD dwResult;
		for (DWORD i = 0; i< XUSER_MAX_COUNT; i++)
		{
			
			ZeroMemory(&state, sizeof(XINPUT_STATE));

			// Simply get the state of the controller from XInput.
			dwResult = XInputGetState(i, &state);

			if (dwResult == ERROR_SUCCESS)
			{
				printf("// Controller is connected %d\n",i);
			}
			else
			{
				printf("// Controller is not connected\n"); 
			}
		}
	}

	//read txt file and get level data
	void loadTxt(int num)
	{
		std::fstream myFile;
		std::stringstream fileName;
		fileName << "level" <<num<< ".txt";
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
			case '_': add_rigid_body(pos, box, floor, c, false);
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'O':add_rigid_body(pos, box, wall, c, false);
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'B':
			case 'F': add_rigid_body(pos, flipperMesh, flip, c, true);
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'E': add_rigid_body(pos, box, end, c, false);
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case 'P': add_rigid_body(pos, box, player, c, true);
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			case '-':
			case '¦': add_rigid_body(pos, box, wall, c, false);
				x += 1;
				pos += vec3(1, 0, 0);
				break;
			default : break;
			}
		}
	}

    /// this is called once OpenGL is initialized
    void app_init() 
	{
      app_scene =  new visual_scene();
	  newScene();
	  loadTxt(2);
    }

    /// this is called to draw the world
    void draw_world(int x, int y, int w, int h) 
	{
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
		  rigid_bodies[player_node]->applyCentralForce(btVector3(0, 100, 0));
	  }

	  else if (is_key_down(key_right))
	  {
		  rigid_bodies[player_node]->applyCentralForce(btVector3(10, 0, 0));
	  }

	  else if (is_key_down(key_left))
	  {
		  rigid_bodies[player_node]->applyCentralForce(btVector3(-10, 0, 0));
	  }

	  if (is_key_going_down('1')||is_key_going_down(VK_NUMPAD1))
	  {
		  newScene();
		  loadTxt(1);
	  }
	  if (is_key_going_down('2') || is_key_going_down(VK_NUMPAD2))
	  {
		  newScene();
		  loadTxt(2);
	  }
	  if (is_key_going_down('3') || is_key_going_down(VK_NUMPAD3))
	  {
		  newScene();
		  loadTxt(3);
	  }

	  //flipper test
	  if (is_key_going_down('F'))
	  {
		  for (int i = 0; i < flippers.size(); i++)
		  {
			  flippers[i]->getRigidBodyA().applyTorqueImpulse(btVector3(0, 0, 50));
		  }
	  }
    }
  };
}
