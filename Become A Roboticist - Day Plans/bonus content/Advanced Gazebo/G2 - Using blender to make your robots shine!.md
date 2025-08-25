## Now you have made a mesh using FreeCAD, it's time to give it some colour.

There are lots of ways to add colour, but the most reliable way I've found is to use Blender to colour the mesh. This embeds the colour in the mesh file, and even lets you experiment with adding textures. Blender is also just a super fun, creative tool, and so I love having an excuse to play with it.

Blender is notorious for having a huge and difficult learning curve. So we will just focus on opening Blender and doing exactly what we need to.

# Open Blender 4.0

Unlike the other programs in this course, we won't use VS Code to open blender.

Instead open it from your OS, you should be presented with this screen:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/lpsyyqt1nzwd92jm0t2fxk6srpdb)

Go File->Import->Collada (.dae)

![](https://s3.us-west-2.amazonaws.com/content.podia.com/xq77s7jj9wa8yq3f208spm86mpse)

Now we need to navigate to your WSL drive. On my computer it looks like this, note I have typed `\\wsl.localhost\Ubuntu\home\johnv\` into the top bar:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/cja22gvwa4gg266q1gj5re3i2hvc)

This string will probably be different for you. To find your WSL folder, open a Windows Finder. Look for a "Linux" drop down in the left panel. Click on Ubuntu.

![](https://s3.us-west-2.amazonaws.com/content.podia.com/vzyla7alfawdg0kcvut1cv4kpqhq)

Now click on "home":

![](https://s3.us-west-2.amazonaws.com/content.podia.com/6sfv3cgxzwuonehelkn3hz1h4ikw)

Inside the home folder, will be usually a single folder of your user name, open it.

![](https://s3.us-west-2.amazonaws.com/content.podia.com/phs23j4gwbs45rhp3t1om1crhwg6)

Inside your home folder you should be able to find the `4robots_ws` folder, open that:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/cynk9o2591u5tcco8q3unmmflfsv)

Inside that folder click on the navigation bar in the middle, and it will turn into a text path. Copy this text path back into your Blender finder window.

![](https://s3.us-west-2.amazonaws.com/content.podia.com/ad3pccj89pv9sykwjlvc1xgbeypy)

Hopefully you will be able to find your saved .dae file somewhere here. This is the `/workspace` folder in your VS Code, but accessible from within windows.

Click on your `.dae` file and select "Import Collada".

![](https://s3.us-west-2.amazonaws.com/content.podia.com/nbwtdg2evci0duywqdv6ngs3s8p1)

# Getting into the right Blender mode

You should see your model, but it may be inside the default cube. Right click the cube in the top right under Scene Collection and click delete to get rid of it. Zoom in to see your model better. Also click the "node0" under collections to highlight your mesh:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/biziz0j0afen455wq2v8u1hvpeju)

Now change to "Edit Mode" by clicking the "Object Mode" box in the top left:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/x12c8iqffas12oucmrvo120qrz4p)

You'll see lines appear on the model, these are the triangular boundaries of our mesh. Now change the view to be "material view" by clicking the symbol in the top right as shown in the next image:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/jlvzohg3te33eqh644ja76wbj8fy)

Once in material view, we will go to the material menu on the left side by clicking the red circle symbol:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/8figwf2dcm5ox271rweypnv9q96d)

Finally lets choose a selection mode. I like to select by faces, so choose the faces option next to the "Edit Mode" drop down:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/ocsfzguj05o88asgbzaievvyejdk)

# Create a new material for your meshes

Each mesh has a "material", which basically means how that mesh reflects light.

In simple terms material basically just means the colour or texture of the surface. This is the part where you can be abit creative with how you colour your mesh. I want to make it look abit wooden, and instead of using a texture I'm just going to use a solid yellow-brown colour. However I'd also like to make the top and bottom parts of it dark blue.

This means I'll need to create two materials, wood and dark blue.

Click the `+` sign to add a new material to the list, and then click the `+ New` button to make it a new material:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/3citlm55qjb1h2n0alodq2r2yxml)

Rename the material by changing the black box. Also click the colour swath to change the Base Colour:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/3adh8hc8ytp8km8gf3yyl7feyjzv)

Do the same steps to create a dark blue:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/sjlvdel6d5mx4m3zc6sxtbt7ngzq)

# Assign Materials To Mesh's

Now the fun part. We need to select the mesh surfaces that we want, and assign them a material. Since we are in face select mode, we just need to select the faces with our drag tool:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/m96qm6z44c6im9wrelkjx96r3mzm)

Once the faces are selected, click "Assign" to assign the material to that mesh's faces:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/8i1h3cyr3tkl28q8arwj28q4pc2r)

Deselect to see the colour in its glory!

![](https://s3.us-west-2.amazonaws.com/content.podia.com/vvrr559d628ahoy4uztczs6c7g5t)

Go through and colour all of your surfaces as you like, assigning the colour that you want. You can also modify the "Body" colour if you just want to change everything instead of specific meshes.

![](https://s3.us-west-2.amazonaws.com/content.podia.com/mkwxmqs3ebl4p3uysbj11cii6z57)

Success! My newly coloured mesh.

Be aware that different rendering systems will show the colours differently (i.e. it will look different in RViz and Gazebo), but atleast you can have a mesh in simulation that looks better than just plain grey!

For extra points you can also experiment with the other material properties, or even adding a texture to your meshes.

# Exporting Your Mesh

Now its time to export your mesh. Feel free to save your blender file too, so you can easily go back and modify it.

Make sure you have the `node0` selected and then go File->Export "Collada (.dae)". We want to export this back into the WSL path, so copy the path you used, or blender may have saved it in the "Recent" box in the bottom left:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/hefkh1n2blvtyr8y1j1ymxvgqlp1)

The right place to save this is going to be inside the 4robots_ws folder, under the path

`src/krytn/robot_description/krytn/meshes/`

There should already be a lidar mesh there:

![](https://s3.us-west-2.amazonaws.com/content.podia.com/jwbzuopm40aj1wqozmjn4cxl40pl)


# Show your work

Publish a post about using Blender and show off your new mesh!