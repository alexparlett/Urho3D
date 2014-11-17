// Animating 3D scene example.
// This sample demonstrates:
//     - Creating a 3D scene and using a script component to animate the objects
//     - Controlling scene ambience with the Zone component
//     - Attaching a light to an object (the camera)

#include "Scripts/Utilities/Sample.as"

Vector3 endPos;
Node@ player;
NavigationCrowdManager@ crowdMng_;

class RandomWalker : ScriptObject
{
    void Update(float timestep)
    {
        NavigationAgent@ a=node.GetComponent("NavigationAgent");
        if(a !is null)
        {
            if(a.targetState == NAV_AGENT_TARGET_ARRIVED)
            {
                Vector3 pos=crowdMng_.navigationMesh.GetRandomPoint();
                a.SetMoveTarget(pos);
            }
        }
    }
}

class Chaser : ScriptObject
{
    Node@ target=null;
    void Update(float timestep)
    {
        NavigationAgent@ a=node.GetComponent("NavigationAgent");
        if(a !is null)
        {
            if(target !is null)
            {
                a.SetMoveTarget(target.position);
            }
        }
    }
    
    void SetTarget(Node@ t)
    {
        target=t;
    }
    
    void Start()
    {
        SubscribeToEvent(node, "NavigationAgentMovement", "HandleReposition");
    }
    
    void HandleReposition(StringHash eventType, VariantMap& eventData)
    {
        Vector3 velocity=eventData["Velocity"].GetVector3();
        Vector3 facing=Vector3(velocity.x, 0, velocity.z);
        facing.Normalize();
        float angle=Atan2(facing.z, facing.x);
        node.rotation=Quaternion(angle-90, Vector3(0,-1,0));
    }
}

void Start()
{
    // Execute the common startup for samples
    SampleStart();

    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateUI();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();
}

Node@ CreatePlayer(Vector3 position, NavigationAvoidanceQuality quality, NavigationPushiness pushiness, float speed, float accel)
{
    Node@ n = scene_.CreateChild("Jack");
    n.position=position;
    AnimatedModel@ modelObject = n.CreateComponent("AnimatedModel");
    modelObject.model=cache.GetResource("Model", "Models/Jack.mdl");
    modelObject.material=cache.GetResource("Material", "Materials/Jack.xml");
    modelObject.castShadows=true;
    AnimationController@ ac=n.CreateComponent("AnimationController");
    ac.Play("Models/Jack_Walk.ani", 0, true, 0.0f);
    NavigationAgent @agent=n.CreateComponent("NavigationAgent");
    agent.maxAccel=accel;
    agent.navigationPushiness=pushiness;
    agent.navigationQuality=quality;
    agent.maxSpeed=speed;
    return n;
}

Node@ CreateChaser(Node@ target, Vector3 position, NavigationAvoidanceQuality quality, NavigationPushiness pushiness, float speed, float accel)
{
    Node@ n = scene_.CreateChild("Jack");
    n.position=position;
    AnimatedModel@ modelObject = n.CreateComponent("AnimatedModel");
    modelObject.model=cache.GetResource("Model", "Models/Jack.mdl");
    modelObject.material=cache.GetResource("Material", "Materials/Jack.xml");
    modelObject.castShadows=true;
    AnimationController@ ac=n.CreateComponent("AnimationController");
    ac.Play("Models/Jack_Walk.ani", 0, true, 0.0f);
    NavigationAgent @agent=n.CreateComponent("NavigationAgent");
    agent.maxAccel=accel;
    agent.navigationPushiness=pushiness;
    agent.navigationQuality=quality;
    agent.maxSpeed=speed;
    Chaser@ c=cast<Chaser>(n.CreateScriptObject(scriptFile, "Chaser", LOCAL));
    c.SetTarget(target);
    return n;
}

void CreateScene()
{
    scene_ = Scene();

    // Create the Octree component to the scene so that drawable objects can be rendered. Use default volume
    // (-1000, -1000, -1000) to (1000, 1000, 1000)
    scene_.CreateComponent("Octree");
    scene_.CreateComponent("DebugRenderer");
    
    // Create scene node & StaticModel component for showing a static plane
    Node@ planeNode = scene_.CreateChild("Plane");
    planeNode.scale = Vector3(100.0f, 1.0f, 100.0f);
    StaticModel@ planeObject = planeNode.CreateComponent("StaticModel");
    planeObject.model = cache.GetResource("Model", "Models/Plane.mdl");
    planeObject.material = cache.GetResource("Material", "Materials/StoneTiled.xml");

    // Create a Zone component for ambient lighting & fog control
    Node@ zoneNode = scene_.CreateChild("Zone");
    Zone@ zone = zoneNode.CreateComponent("Zone");
    zone.boundingBox = BoundingBox(-1000.0f, 1000.0f);
    zone.ambientColor = Color(0.15f, 0.15f, 0.15f);
    zone.fogColor = Color(0.5f, 0.5f, 0.7f);
    zone.fogStart = 100.0f;
    zone.fogEnd = 300.0f;

    // Create a directional light to the world. Enable cascaded shadows on it
    Node@ lightNode = scene_.CreateChild("DirectionalLight");
    lightNode.direction = Vector3(0.6f, -1.0f, 0.8f);
    Light@ light = lightNode.CreateComponent("Light");
    light.lightType = LIGHT_DIRECTIONAL;
    light.castShadows = true;
    light.shadowBias = BiasParameters(0.00025f, 0.5f);
    // Set cascade splits at 10, 50 and 200 world units, fade shadows out at 80% of maximum shadow distance
    light.shadowCascade = CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f);

    // Create some mushrooms
    const uint NUM_MUSHROOMS = 100;
    for (uint i = 0; i < NUM_MUSHROOMS; ++i)
        CreateMushroom(Vector3(Random(90.0f) - 45.0f, 0.0f, Random(90.0f) - 45.0f));

    // Create randomly sized boxes. If boxes are big enough, make them occluders. Occluders will be software rasterized before
    // rendering to a low-resolution depth-only buffer to test the objects in the view frustum for visibility
    const uint NUM_BOXES = 20;
    for (uint i = 0; i < NUM_BOXES; ++i)
    {
        Node@ boxNode = scene_.CreateChild("Box");
        float size = 1.0f + Random(10.0f);
        boxNode.position = Vector3(Random(80.0f) - 40.0f, size * 0.5f, Random(80.0f) - 40.0f);
        boxNode.SetScale(size);
        StaticModel@ boxObject = boxNode.CreateComponent("StaticModel");
        boxObject.model = cache.GetResource("Model", "Models/Box.mdl");
        boxObject.material = cache.GetResource("Material", "Materials/Stone.xml");
        boxObject.castShadows = true;
        if (size >= 3.0f)
            boxObject.occluder = true;
    }
   
    

    // Create a NavigationMesh component to the scene root
    NavigationMesh@ navMesh = scene_.CreateComponent("NavigationMesh");
    // Create a Navigable component to the scene root. This tags all of the geometry in the scene as being part of the
    // navigation mesh. By default this is recursive, but the recursion could be turned off from Navigable
    scene_.CreateComponent("Navigable");
    // Add padding to the navigation mesh in Y-direction so that we can add objects on top of the tallest boxes
    // in the scene and still update the mesh correctly
    navMesh.padding=Vector3(0.0f, 10.0f, 0.0f);
    // Now build the navigation geometry. This will take some time. Note that the navigation mesh will prefer to use
    // physics geometry from the scene nodes, as it often is simpler, but if it can not find any (like in this example)
    // it will use renderable geometry instead
    navMesh.Build();

    // Create the camera. Limit far clip distance to match the fog
    cameraNode = scene_.CreateChild("Camera");
    Camera@ camera = cameraNode.CreateComponent("Camera");
    camera.farClip = 300.0f;

    // Set an initial position for the camera scene node above the plane
    cameraNode.position = Vector3(0.0f, 25.0f, 0.0f);

    // Create a DetourCrowdManager component to the scene root
    crowdMng_ = scene_.CreateComponent("NavigationCrowdManager");
    
    // Now build the crowd.
    if(crowdMng_.CreateCrowd())
    {
        // Create the player object
        player=CreatePlayer(Vector3(0, 0, 0), NAVIGATIONQUALITY_HIGH, PUSHINESS_HIGH, 10, 100.0f);
        
        for(uint i=0; i<20; ++i)
        {
            CreateChaser(player, Vector3(Random(80.0f)-40.0f, 0, Random(80.0f)-40.0f), NAVIGATIONQUALITY_HIGH, PUSHINESS_LOW, 5.f, 100.0f);
        }
    }
}

void CreateUI()
{
    // Create a Cursor UI element because we want to be able to hide and show it at will. When hidden, the mouse cursor will
    // control the camera, and when visible, it will point the raycast target
    XMLFile@ style = cache.GetResource("XMLFile", "UI/DefaultStyle.xml");
    Cursor@ cursor = Cursor();
    cursor.SetStyleAuto(style);
    ui.cursor = cursor;
    // Set starting position of the cursor at the rendering window center
    cursor.SetPosition(graphics.width / 2, graphics.height / 2);

    // Construct new Text object, set string to display and font to use
    Text@ instructionText = ui.root.CreateChild("Text");
    instructionText.text =
        "Use WASD keys to move, RMB to rotate view\n"
        "LMB to set destination, SHIFT+LMB to teleport\n"
        "MMB to add or remove obstacles\n"
        "Space to toggle debug geometry";
    instructionText.SetFont(cache.GetResource("Font", "Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    instructionText.textAlignment = HA_CENTER;

    // Position the text relative to the screen center
    instructionText.horizontalAlignment = HA_CENTER;
    instructionText.verticalAlignment = VA_CENTER;
    instructionText.SetPosition(0, ui.root.height / 4);
}

void SetupViewport()
{
    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    Viewport@ viewport = Viewport(scene_, cameraNode.GetComponent("Camera"));
    renderer.viewports[0] = viewport;
}

void SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent("Update", "HandleUpdate");

    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request
    // debug geometry
    SubscribeToEvent("PostRenderUpdate", "HandlePostRenderUpdate");
}

void MoveCamera(float timeStep)
{
    // Right mouse button controls mouse cursor visibility: hide when pressed
    ui.cursor.visible = !input.mouseButtonDown[MOUSEB_RIGHT];

    // Do not move if the UI has a focused element (the console)
    if (ui.focusElement !is null)
        return;

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    // Only move the camera when the cursor is hidden
    if (!ui.cursor.visible)
    {
        IntVector2 mouseMove = input.mouseMove;
        yaw += MOUSE_SENSITIVITY * mouseMove.x;
        pitch += MOUSE_SENSITIVITY * mouseMove.y;
        pitch = Clamp(pitch, -90.0f, 90.0f);

        // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
        cameraNode.rotation = Quaternion(pitch, yaw, 0.0f);
    }

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input.keyDown['W'])
        cameraNode.Translate(Vector3(0.0f, 0.0f, 1.0f) * MOVE_SPEED * timeStep);
    if (input.keyDown['S'])
        cameraNode.Translate(Vector3(0.0f, 0.0f, -1.0f) * MOVE_SPEED * timeStep);
    if (input.keyDown['A'])
        cameraNode.Translate(Vector3(-1.0f, 0.0f, 0.0f) * MOVE_SPEED * timeStep);
    if (input.keyDown['D'])
        cameraNode.Translate(Vector3(1.0f, 0.0f, 0.0f) * MOVE_SPEED * timeStep);

    // Set destination or teleport with left mouse button
    if (input.mouseButtonDown[MOUSEB_LEFT])
        SetPathPoint();
    // Add or remove objects with middle mouse button, then rebuild navigation mesh partially
    if (input.mouseButtonPress[MOUSEB_MIDDLE])
        AddOrRemoveObject();

    // Toggle debug geometry with space
    if (input.keyPress[KEY_SPACE])
        drawDebug = !drawDebug;
}

void SetPathPoint()
{
    Vector3 hitPos;
    Drawable@ hitDrawable;
    NavigationMesh@ navMesh = scene_.GetComponent("NavigationMesh");

    if (Raycast(250.0f, hitPos, hitDrawable))
    {
        Vector3 pathPos = navMesh.FindNearestPoint(hitPos, Vector3(1.0f, 1.0f, 1.0f));
        NavigationAgent@ playerAgent=player.GetComponent("NavigationAgent");
        playerAgent.SetMoveTarget(pathPos);
    }
}

void AddOrRemoveObject()
{
    // Raycast and check if we hit a mushroom node. If yes, remove it, if no, create a new one
    Vector3 hitPos;
    Drawable@ hitDrawable;

    if (Raycast(250.0f, hitPos, hitDrawable))
    {
        // The part of the navigation mesh we must update, which is the world bounding box of the associated
        // drawable component
        BoundingBox updateBox;

        Node@ hitNode = hitDrawable.node;
        if (hitNode.name == "Mushroom")
        {
            updateBox = hitDrawable.worldBoundingBox;
            hitNode.Remove();
        }
        else
        {
            Node@ newNode = CreateMushroom(hitPos);
            StaticModel@ newObject = newNode.GetComponent("StaticModel");
            updateBox = newObject.worldBoundingBox;
        }

        // Rebuild part of the navigation mesh, then rebuild the path if applicable
        NavigationMesh@ navMesh = scene_.GetComponent("NavigationMesh");
        navMesh.Build(updateBox);
    }
}

Node@ CreateMushroom(const Vector3& pos)
{
    Node@ mushroomNode = scene_.CreateChild("Mushroom");
    mushroomNode.position = pos;
    mushroomNode.rotation = Quaternion(0.0f, Random(360.0f), 0.0f);
    mushroomNode.SetScale(2.0f + Random(0.5f));
    StaticModel@ mushroomObject = mushroomNode.CreateComponent("StaticModel");
    mushroomObject.model = cache.GetResource("Model", "Models/Mushroom.mdl");
    mushroomObject.material = cache.GetResource("Material", "Materials/Mushroom.xml");
    mushroomObject.castShadows = true;

    return mushroomNode;
}

bool Raycast(float maxDistance, Vector3& hitPos, Drawable@& hitDrawable)
{
    hitDrawable = null;

    IntVector2 pos = ui.cursorPosition;
    // Check the cursor is visible and there is no UI element in front of the cursor
    if (!ui.cursor.visible || ui.GetElementAt(pos, true) !is null)
        return false;

    Camera@ camera = cameraNode.GetComponent("Camera");
    Ray cameraRay = camera.GetScreenRay(float(pos.x) / graphics.width, float(pos.y) / graphics.height);
    // Pick only geometry objects, not eg. zones or lights, only get the first (closest) hit
    // Note the convenience accessor to scene's Octree component
    RayQueryResult result = scene_.octree.RaycastSingle(cameraRay, RAY_TRIANGLE, maxDistance, DRAWABLE_GEOMETRY);
    if (result.drawable !is null)
    {
        hitPos = result.position;
        hitDrawable = result.drawable;
        return true;
    }

    return false;
}

void HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    // Take the frame time step, which is stored as a float
    float timeStep = eventData["TimeStep"].GetFloat();

    // Move the camera, scale movement with time step
    MoveCamera(timeStep);

    
}

void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enabled, draw navigation mesh debug geometry
    if (drawDebug)
    {
        DebugRenderer@ dbg=scene_.GetComponent("DebugRenderer");
        NavigationMesh@ navMesh = scene_.GetComponent("NavigationMesh");
        navMesh.DrawDebugGeometry(true);
        crowdMng_.DrawDebug(dbg, true);
    }
}

// Create XML patch instructions for screen joystick layout specific to this sample app
String patchInstructions =
        "<patch>" +
        "    <add sel=\"/element\">" +
        "        <element type=\"Button\">" +
        "            <attribute name=\"Name\" value=\"Button3\" />" +
        "            <attribute name=\"Position\" value=\"-120 -120\" />" +
        "            <attribute name=\"Size\" value=\"96 96\" />" +
        "            <attribute name=\"Horiz Alignment\" value=\"Right\" />" +
        "            <attribute name=\"Vert Alignment\" value=\"Bottom\" />" +
        "            <attribute name=\"Texture\" value=\"Texture2D;Textures/TouchInput.png\" />" +
        "            <attribute name=\"Image Rect\" value=\"96 0 192 96\" />" +
        "            <attribute name=\"Hover Image Offset\" value=\"0 0\" />" +
        "            <attribute name=\"Pressed Image Offset\" value=\"0 0\" />" +
        "            <element type=\"Text\">" +
        "                <attribute name=\"Name\" value=\"Label\" />" +
        "                <attribute name=\"Horiz Alignment\" value=\"Center\" />" +
        "                <attribute name=\"Vert Alignment\" value=\"Center\" />" +
        "                <attribute name=\"Color\" value=\"0 0 0 1\" />" +
        "                <attribute name=\"Text\" value=\"Teleport\" />" +
        "            </element>" +
        "            <element type=\"Text\">" +
        "                <attribute name=\"Name\" value=\"KeyBinding\" />" +
        "                <attribute name=\"Text\" value=\"LSHIFT\" />" +
        "            </element>" +
        "            <element type=\"Text\">" +
        "                <attribute name=\"Name\" value=\"MouseButtonBinding\" />" +
        "                <attribute name=\"Text\" value=\"LEFT\" />" +
        "            </element>" +
        "        </element>" +
        "        <element type=\"Button\">" +
        "            <attribute name=\"Name\" value=\"Button4\" />" +
        "            <attribute name=\"Position\" value=\"-120 -12\" />" +
        "            <attribute name=\"Size\" value=\"96 96\" />" +
        "            <attribute name=\"Horiz Alignment\" value=\"Right\" />" +
        "            <attribute name=\"Vert Alignment\" value=\"Bottom\" />" +
        "            <attribute name=\"Texture\" value=\"Texture2D;Textures/TouchInput.png\" />" +
        "            <attribute name=\"Image Rect\" value=\"96 0 192 96\" />" +
        "            <attribute name=\"Hover Image Offset\" value=\"0 0\" />" +
        "            <attribute name=\"Pressed Image Offset\" value=\"0 0\" />" +
        "            <element type=\"Text\">" +
        "                <attribute name=\"Name\" value=\"Label\" />" +
        "                <attribute name=\"Horiz Alignment\" value=\"Center\" />" +
        "                <attribute name=\"Vert Alignment\" value=\"Center\" />" +
        "                <attribute name=\"Color\" value=\"0 0 0 1\" />" +
        "                <attribute name=\"Text\" value=\"Obstacles\" />" +
        "            </element>" +
        "            <element type=\"Text\">" +
        "                <attribute name=\"Name\" value=\"MouseButtonBinding\" />" +
        "                <attribute name=\"Text\" value=\"MIDDLE\" />" +
        "            </element>" +
        "        </element>" +
        "    </add>" +
        "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]/attribute[@name='Is Visible']\" />" +
        "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Set</replace>" +
        "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button0']]\">" +
        "        <element type=\"Text\">" +
        "            <attribute name=\"Name\" value=\"MouseButtonBinding\" />" +
        "            <attribute name=\"Text\" value=\"LEFT\" />" +
        "        </element>" +
        "    </add>" +
        "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/attribute[@name='Is Visible']\" />" +
        "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Debug</replace>" +
        "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]\">" +
        "        <element type=\"Text\">" +
        "            <attribute name=\"Name\" value=\"KeyBinding\" />" +
        "            <attribute name=\"Text\" value=\"SPACE\" />" +
        "        </element>" +
        "    </add>" +
        "</patch>";