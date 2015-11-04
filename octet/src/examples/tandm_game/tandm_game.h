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

	//XINPUT GAMEPAD
	static const WORD GAMEPADBUTTONS[] = {
		XINPUT_GAMEPAD_A,
		XINPUT_GAMEPAD_B,
		XINPUT_GAMEPAD_X,
		XINPUT_GAMEPAD_Y,
		XINPUT_GAMEPAD_DPAD_UP,
		XINPUT_GAMEPAD_DPAD_DOWN,
		XINPUT_GAMEPAD_DPAD_LEFT,
		XINPUT_GAMEPAD_DPAD_RIGHT,
		XINPUT_GAMEPAD_LEFT_SHOULDER,
		XINPUT_GAMEPAD_RIGHT_SHOULDER,
		XINPUT_GAMEPAD_LEFT_THUMB,
		XINPUT_GAMEPAD_RIGHT_THUMB,
		XINPUT_GAMEPAD_BACK,
		XINPUT_GAMEPAD_START
	};

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
	btRigidBody *playerRB;
	dynarray<btRigidBody*> rigid_bodies;
	dynarray<btHingeConstraint*> flippers;
	dynarray<btGeneric6DofSpringConstraint*> springBodies;
	mesh_box *box, *flipperMesh, *blockerMesh;
	mesh_sphere *sph;
	material *wall, *floor, *flip, *end, *player, *invisWall;

	scene_node *cam;

	//string to hold the txt file
	string contents;

	//Hinge variables
	bool hingeOffsetNotSet = false;
	btVector3 offset;

	//Xbox controller enums and state
	XINPUT_STATE state;
	DWORD dwResult;

	enum BUTTONS {
		FaceA=0,
		B,
		X,
		Y,
		Up,
		Down,
		Left,
		Right,
		LeftShoulder,
		RightShoulder,
		LeftStick,
		RightStick,
		Start,
		Back
	};

	float left = Left;
	float right = Right;
	float up = Up;
	float down = Down;
	float faceA = FaceA;
	float start = Start;
	float back = Back;

	bool controllerConnected = false;

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
		
		//rigid_bodies.back()->setFriction(0);
		//rigid_bodies.back()->setRestitution(0);

		//static object 
		if (letter == 'O')
		{
			staticObject = rigid_bodies.back();
		}
		
		//player object
		if (letter == 'P')
		{
			playerRB = rigid_bodies.back();
			playerRB->setDamping(0, 0);
			playerRB->setLinearFactor(btVector3(1, 1, 0)); //constraints z axis movement
			playerRB->setAngularFactor(btVector3(0, 0, 1)); //constraints x and y axis rotation
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

		//Hinges
		if (letter == 'F')
		{
			if (!hingeOffsetNotSet)
			{
				offset = get_btVector3(flipperMesh->get_aabb().get_max());
				offset = btVector3(offset.x()*0.5f, offset.y()*0.0f, offset.z()*0.0f);
				printf("%g %g %g\n", offset.x(), offset.y(), offset.z());
				hingeOffsetNotSet = true;
			}
			
			btRigidBody *hingeBody = rigid_bodies.back();
			btHingeConstraint *flipperHinge =new btHingeConstraint(*hingeBody, offset, btVector3(0, 0, 1));
			world->addConstraint(flipperHinge);
			flippers.push_back(flipperHinge);
		}

		worldCoord.loadIdentity();
	}

	//Controller functions
	void controller()
	{
		//Xbox Controller detection code
		ZeroMemory(&state, sizeof(XINPUT_STATE));

		// Simply get the state of the controller from XInput.
		dwResult = XInputGetState(0, &state);

		if (dwResult == ERROR_SUCCESS)
		{
			printf("Controller is connected\n");
			controllerConnected = true;
		}
		else
		{
			printf("Controller is not connected\n");
		}
	}

	//check state if controller is unplugged
	XINPUT_STATE getState()
	{
		XINPUT_STATE currentState;
		ZeroMemory(&currentState, sizeof(XINPUT_STATE));
		dwResult = XInputGetState(0, &currentState);
		if (dwResult == ERROR_SUCCESS)
		{
			controllerConnected = true;
		}
		else
		{
			controllerConnected = false;
		}
		return currentState;
	}

	void controllerUpdate()
	{
		state = getState();
	}
	
	bool buttonPress(int button)
	{
		return (state.Gamepad.wButtons & GAMEPADBUTTONS[button]) ? true : false;
	}

	//Collison check
	void collided()
	{

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
		image *transparentImg = new image("assets/transpparent.jpg");
		image * transparentMask = new image("assets/transparent.gif");
		param_shader *transparentShader = new param_shader("shaders/default.vs", "shaders/multitexture.fs");
		wall = new material(vec4(1, 0, 0, 1));
		invisWall = new material(vec4(1,1,1,1),transparentShader);
		floor = new material(vec4(0, 1, 0, 1));
		flip = new material(vec4(0, 0, 1, 1));
		end = new material(vec4(1, 1, 1, 1));
		player = new material(vec4(0, 1, 1, 0));
		offset = btVector3(0, 0, 0);
		controller();
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
			case '¦': add_rigid_body(pos, box, invisWall, c, false);
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

	  //Controller
	  controllerUpdate();

	  if (controllerConnected)
	  {
		  if (buttonPress(left))
		  {
			  playerRB->applyCentralForce(btVector3(-10, 0, 0));
		  }

		  else if (buttonPress(right))
		  {
			  playerRB->applyCentralForce(btVector3(10, 0, 0));
		  }

		  else if (buttonPress(FaceA))
		  {
			  playerRB->applyCentralForce(btVector3(0, 25, 0));
		  }
		  else
		  {
			  playerRB->setFriction(1.0);
		  }
	  }

	  //Keyboard
	  else if (!controllerConnected)
	  {
		  //KEY INPUTS
		  if (is_key_down(VK_SPACE))
		  {
			  playerRB->applyCentralForce(btVector3(0, 25, 0));
		  }

		  else if (is_key_down(key_right))
		  {
			  playerRB->applyCentralForce(btVector3(10, 0, 0));
		  }

		  else if (is_key_down(key_left))
		  {
			  playerRB->applyCentralForce(btVector3(-10, 0, 0));
		  }

		  else if (is_key_going_down('S'))
		  {
			  cam->loadIdentity();
			  cam->rotate(90, vec3(0, -1, 0));
			  cam->translate(vec3(0,-46,-10));
		  }

		  else if (is_key_down('G'))
		  {
			  
		  }
		  else
		  {
			  playerRB->setFriction(1.0);
		  }
	  }

	  /*if (is_key_going_down('1')||is_key_going_down(VK_NUMPAD1))
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
	  }*/
    }
  };
}
