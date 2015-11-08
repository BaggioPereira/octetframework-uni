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

	//World coordinates
	mat4t worldCoord;

	//rigid bodies
	btRigidBody *staticObject;
	btRigidBody *playerRB;

	//dynarrays
	dynarray<btRigidBody*> rigid_bodies;
	dynarray<btHingeConstraint*> flippers;
	dynarray<btGeneric6DofSpringConstraint*> springBodies;
	dynarray<scene_node*> nodes;

	//meshes
	mesh_box *box, *flipperMesh, *blockerMesh;

	//materials
	material *wall, *floor, *flip, *end, *player, *invisWall;

	scene_node *cam, *playerNode;
	mat4t camToWorld;

	enum objs {
		PLAYER = 0,
		BLOCKER,
		FLIPPER
	};

	//prepection booleans
	bool third = true;
	bool first = false;
	bool flipped = false;

	//string to hold the txt file
	string contents;
	bool multipart = false;

	//Hinge variables
	bool hingeOffsetNotSet = false;
	btVector3 offset;

	//Xbox controller enums and state
	XINPUT_STATE state;
	DWORD dwResult;

	//enums for controller
	enum BUTTONS {
		FaceA=0,
		FaceB,
		FaceX,
		FaceY,
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

	bool controllerConnected = false;

	bool yButton = false, aButton = false, xButton = false;

	//Sounds
	ALuint boing;
	ALuint jump;
	unsigned cur_source;
	ALuint sources[2];

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
	void add_part(mat4t_in coord, mesh *shape, material *mat, bool is_dynamic,char letter)
	{
		scene_node *node = new scene_node();
		node->access_nodeToParent() = coord;
		app_scene->add_child(node);
		app_scene->add_mesh_instance(new mesh_instance(node, shape, mat));
		nodes.push_back(node);

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
			//rigid_body->setUserPointer(node);
			if (letter == 'P')
			{
				
			}
			else if (letter == 'B')
			{
				
			}
			else if (letter == 'F')
			{
				
			}
			
		}
	}

	//call to create mesh and set position with friction and restitution
	void add_rigid_body(vec3 position, mesh *msh, material *mat, char letter, bool active)
	{
		worldCoord.translate(position);
		add_part(worldCoord, msh, mat, active,letter);
		
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
			playerNode = nodes.back();
			playerRB->setUserIndex(PLAYER);
			int index = playerRB->getUserIndex();
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
			springBody->setUserIndex(BLOCKER);
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
				hingeOffsetNotSet = true;
			}
			
			btRigidBody *hingeBody = rigid_bodies.back();
			hingeBody->setUserIndex(FLIPPER);
			btHingeConstraint *flipperHinge =new btHingeConstraint(*hingeBody, offset, btVector3(0, 0, 1));
			flipperHinge->enableAngularMotor(true, 10, 1000);
			world->addConstraint(flipperHinge);
			flippers.push_back(flipperHinge);
		}

		worldCoord.loadIdentity();
	}

	//Controller functions
	/*void controller()
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
	}*/

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

	//movement code
	void controls(bool controller)
	{
		if (controller)
		{
			if (third)
			{
				playerRB->setLinearFactor(btVector3(1, 1, 0)); 
				playerRB->setAngularFactor(btVector3(0, 0, 1));
				if (buttonPress(Left))
				{
					playerRB->activate();
					playerRB->applyCentralForce(btVector3(-10, 0, 0));
				}

				else if (buttonPress(Right))
				{
					playerRB->activate();
					playerRB->applyCentralForce(btVector3(10, 0, 0));
				}
			}

			else if (first)
			{
				playerRB->setLinearFactor(btVector3(1, 1, 1));
				playerRB->setAngularFactor(btVector3(1, 0, 1));
				if (buttonPress(Down))
				{
					playerRB->activate();
					if (!flipped)
						playerRB->applyCentralForce(btVector3(-10, 0, 0));
					else
						playerRB->applyCentralForce(btVector3(10, 0, 0));
				}

				else if (buttonPress(Up))
				{
					playerRB->activate();
					if (!flipped)
						playerRB->applyCentralForce(btVector3(10, 0, 0));
					else
						playerRB->applyCentralForce(btVector3(-10, 0, 0));
				}

				else if (buttonPress(Left))
				{
					playerRB->activate();
					vec3 pos = playerNode->get_position();
					if (pos.x()>-0.1f)
						playerRB->applyCentralForce(btVector3(0, 0, -10));
					else
						playerRB->applyCentralForce(btVector3(0, 0, 0));
				}
				else if (buttonPress(Right))
				{
					playerRB->activate();
					vec3 pos = playerNode->get_position();
					printf("%g %g %g\n", pos.x(), pos.y(), pos.z());
					if (pos.z()<0.1f)
						playerRB->applyCentralForce(btVector3(0, 0, 10));
					else
						playerRB->applyCentralForce(btVector3(0, 0, 0));
				}
			}
			
			if (aButton)
			{
				playerRB->activate();
				playerRB->applyCentralForce(btVector3(0, 100, 0));
				ALuint source = get_sound_source();
				alSourcei(source, AL_BUFFER, jump);
				alSourcePlay(source);
			}

			if (yButton)
			{
				if (third)
				{
					playerRB->clearForces();
					third = !third;
					first = !first;
				}
				else if (!third)
				{
					playerRB->clearForces();
					third = !third;
					first = !first;
				}
			}

			if (first)
			{
				if (xButton)
				{
					flipped = !flipped;
				}
			}

			else
			{
				playerRB->setFriction(1.0);
			}

		}

		else if (!controller)
		{
			//KEYBOARD INPUTS
			if (third)
			{
				playerRB->setLinearFactor(btVector3(1, 1, 0));
				playerRB->setAngularFactor(btVector3(0, 0, 1));
				if (is_key_down(key_right))
				{
					playerRB->activate();
					playerRB->applyCentralForce(btVector3(10, 0, 0));
				}

				else if (is_key_down(key_left))
				{
					playerRB->activate();
					playerRB->applyCentralForce(btVector3(-10, 0, 0));
				}
			}

			else if (first)
			{
				playerRB->setLinearFactor(btVector3(1, 1, 1));
				playerRB->setAngularFactor(btVector3(1, 0, 1));
				if (is_key_down(key_up))
				{
					playerRB->activate();
					if (!flipped)
						playerRB->applyCentralForce(btVector3(10, 0, 0));
					else
						playerRB->applyCentralForce(btVector3(-10, 0, 0));
				}

				else if (is_key_down(key_down))
				{
					playerRB->activate();
					if (!flipped)
						playerRB->applyCentralForce(btVector3(-10, 0, 0));
					else
						playerRB->applyCentralForce(btVector3(10, 0, 0));
				}

				else if (is_key_going_down(key_left))
				{
					playerRB->activate();
					vec3 pos = playerNode->get_position();
					if (pos.x()>-0.1f)
						playerRB->applyCentralForce(btVector3(0, 0, -10));
					else
						playerRB->applyCentralForce(btVector3(0, 0, 0));
				}
				else if (is_key_going_down(key_right))
				{
					playerRB->activate();
					vec3 pos = playerNode->get_position();
					printf("%g %g %g\n", pos.x(), pos.y(), pos.z());
					if (pos.z()<0.1f)
						playerRB->applyCentralForce(btVector3(0, 0, 10));
					else
						playerRB->applyCentralForce(btVector3(0, 0, 0));
				}
			}

			if (is_key_going_down(VK_SPACE))
			{
				playerRB->activate();
				playerRB->applyCentralForce(btVector3(0, 100, 0));
				ALuint source = get_sound_source();
				alSourcei(source, AL_BUFFER, jump);
				alSourcePlay(source);
			}

			if (is_key_going_down('S'))
			{
				if (third)
				{
					playerRB->clearForces();
					third = !third;
					first = !first;
				}
				else if (!third)
				{
					playerRB->clearForces();
					third = !third;
					first = !first;
				}
			}

			if (first)
			{
				if (is_key_going_down('F'))
				{
					flipped = !flipped;
				}
			}

			else
			{
				playerRB->setFriction(1.0);
			}
		}
	}

	//controller button delay
	void buttonDelay()
	{
		if (!yButton)
		{
			yButton = state.Gamepad.wButtons & GAMEPADBUTTONS[FaceY];
		}

		else if (yButton)
		{
			yButton = !state.Gamepad.wButtons & GAMEPADBUTTONS[FaceY];
		}	

		if (!aButton)
		{
			aButton = state.Gamepad.wButtons & GAMEPADBUTTONS[FaceA];
		}

		else if (aButton)
		{
			aButton = !state.Gamepad.wButtons & GAMEPADBUTTONS[FaceA];
		}

		if (!xButton)
		{
			xButton = state.Gamepad.wButtons & GAMEPADBUTTONS[FaceX];
		}

		else if (xButton)
		{
			xButton = !state.Gamepad.wButtons & GAMEPADBUTTONS[FaceX];
		}
	}

	//Sound
	//taken from Andys Invaderers example
	ALuint get_sound_source()
	{
		return sources[cur_source++ % 8];
	}

	//clear the scene
	void newScene()
	{
		rigid_bodies.reset();
		flippers.reset();
		springBodies.reset();
		nodes.reset();
		app_scene->reset();
		app_scene->create_default_camera_and_lights();
		app_scene->get_camera_instance(0)->set_far_plane(1000);
		cam = app_scene->get_camera_instance(0)->get_node();
		cam->translate(vec3(24, -24, 50));
		jump = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "src/examples/tandm_game/jump.wav");
		boing = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "src/examples/tandm_game/hit.wav");
		cur_source = 0;
		alGenSources(8, sources);
		box = new mesh_box(0.5f);
		flipperMesh = new mesh_box(vec3(0.5f, 0.25f, 0.25f));
		blockerMesh = new mesh_box(vec3(0.5f, 1, 0.25f));
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
		printf("Keyboard Controls\n");
		printf("Up arrow to move forward in 1st Person\n");
		printf("Down arrow to move backward in 1st Person\n");
		printf("Left arrow to move left in 3st Person\n");
		printf("Right arrow to move right in 3st Person\n");
		printf("Space to jump\n");
		printf("S Key to switch from 3rd Person to 1st Person and vice-versa\n");
		printf("F Key to flip in 1st Person\n");

		printf("\n\n");
		printf("Controller Controls\n");
		printf("DPad Up to move forward in 1st Person\n");
		printf("DPad Down to move backward in 1st Person\n");
		printf("DPad Left to move left in 3st Person\n");
		printf("DPad Right to move right in 3st Person\n");
		printf("A to jump\n");
		printf("Y to switch from 3rd Person to 1st Person and vice-versa\n");
		printf("X to flip in 1st Person\n");
	}

	//read txt file and get level data
	void loadTxt()
	{
		std::fstream myFile;
		std::stringstream fileName;
		fileName << "level.txt";
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
			levelCreate();
		}
	}

	int findPosition(char letter,int start)
	{
		for (int i = start; i < contents.size(); i++)
		{
			if (contents[i] == letter)
			{
				return i;
			}
		}
		return -1;
	}

	//check to see if level is 3D
	void levelCreate()
	{
		if (contents.find("MultiPart") == -1)
		{
			createLevelArea(0, contents.size(), 0);
		}

		else
		{
			int startLoc = contents.find("Mid") + 3;
			int endLoc = findPosition(';', startLoc);
			printf("%d\n", startLoc);
			createLevelArea(startLoc, endLoc, 0);


			startLoc = contents.find("Front") + 5;
			endLoc = findPosition(';', startLoc);
			createLevelArea(startLoc, endLoc, 1);

			startLoc = contents.find("Back") + 4;
			endLoc = findPosition(';', startLoc);
			createLevelArea(startLoc, endLoc, -1);
		}			
	}

	//create the level according to the txt file
	void createLevelArea(int startLoc, int endLoc, int z)
	{
		vec3 pos = vec3(0, 0, z);
		int x = 0;
		for (int i = startLoc; i < endLoc; i++)
		{
			char c = contents[i];
			switch (c)
			{
			case '\n': pos -= (vec3(x, 0, 0));
				x = 0;
				pos += vec3(0, -1, 0);
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
			default : 
				x += 1;
				pos += vec3(1, 0, 0); 
				break;
			}
		}
	}

    /// this is called once OpenGL is initialized
    void app_init() 
	{
      app_scene =  new visual_scene();
	  newScene();
	  loadTxt();
    }

    /// this is called to draw the world
    void draw_world(int x, int y, int w, int h) 
	{
      int vx = 0, vy = 0;
      get_viewport_size(vx, vy);
      app_scene->begin_render(vx, vy);

	  //Collision checks
	  int manifolds = world->getDispatcher()->getNumManifolds();
	  for (int i = 0; i < manifolds; i++)
	  {
		  btPersistentManifold* contact = world->getDispatcher()->getManifoldByIndexInternal(i);
		  int obj1 = contact->getBody0()->getUserIndex();
		  int obj2 = contact->getBody1()->getUserIndex();
		  if (obj1 == PLAYER)
		  {
			  if (obj2 == BLOCKER)
			  {
				  ALuint source = get_sound_source();
				  alSourcei(source, AL_BUFFER, boing);
				  alSourcePlay(source);
			  }

			  else if (obj2 == FLIPPER)
			  {
				  ALuint source = get_sound_source();
				  alSourcei(source, AL_BUFFER, boing);
				  alSourcePlay(source);
			  }
		  }
	  }
	  
	  world->stepSimulation(1.0f / 30, 1.0f / 30, 1.0f / 30);
	  //Physics setup
	  for (unsigned i = 0; i != rigid_bodies.size(); ++i) {
		  btRigidBody *rigid_body = rigid_bodies[i];
		  btQuaternion btq = rigid_body->getOrientation();
		  btVector3 pos = rigid_body->getCenterOfMassPosition();
		  quat q(btq[0], btq[1], btq[2], btq[3]);
		  mat4t modelToWorld = q;
		  modelToWorld[3] = vec4(pos[0], pos[1], pos[2], 1);
		  nodes[i]->access_nodeToParent() = modelToWorld;
	  }

      // update matrices. assume 30 fps.
      app_scene->update(1.0f/30);

      // draw the scene
      app_scene->render((float)vx / vy);

	  //Controller
	  controllerUpdate();
	  controls(controllerConnected);
	  buttonDelay();

	  //Camera update
	  if (third)
	  {
		  cam->loadIdentity();
		  camToWorld = cam->access_nodeToParent();
		  camToWorld.w() = (playerNode->get_position() + vec3(0, 0, 50.0f)).xyz1();
		  cam->translate(vec3(camToWorld.w().x(), camToWorld.w().y(), camToWorld.w().z()));
	  }

	  else if (first)
	  {
		  cam->loadIdentity();
		  if (!flipped)
		  {
			  cam->rotate(90, vec3(0, -1, 0));
			  camToWorld = cam->access_nodeToParent();
			  camToWorld.w() = (playerNode->get_position() + vec3(-4.5f, 1.25f, 0.0f)).xyz1();
			  cam->translate(vec3(camToWorld.w().z(), camToWorld.w().y(), -camToWorld.w().x()));
		  }
		  else if (flipped)
		  {
			  cam->rotate(90, vec3(0, 1, 0));
			  camToWorld = cam->access_nodeToParent();
			  camToWorld.w() = (playerNode->get_position() + vec3(4.5f, 1.25f, 0.0f)).xyz1();
			  cam->translate(vec3(camToWorld.w().z(), camToWorld.w().y(), camToWorld.w().x()));
		  }
	  }
    }
  };
}
