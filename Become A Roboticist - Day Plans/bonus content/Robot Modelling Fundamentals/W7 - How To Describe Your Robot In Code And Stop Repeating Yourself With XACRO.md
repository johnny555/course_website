URDF, or Universal Robot Description Format is a simple format for describing robots. 

It has 2 main tags, `link` and `joint`, but the user can put other tags in as well. 

XACRO is an XML macro language, it lets you import XML documents and create macros to make creating tedious XML docs faster. We usually pair xacro with URDF because the underlying URDF can be tedious, time consuming or error prone to write by hand. 

A big problem with XML is that the syntax is very picky. It's easy to make mistakes such as the following 2 common bugs: 
## Unmatched Tags

```xml
<link name=”link_1">
<link name=”link_2"> </link>
<link name=”link_3"/>
```

The above snippet will result in an error. This is because the `link_1` doesn't have an end tag. It should have an ending tag with a `/` like `link_2` or `link_3` has. 

## Forgetting End Quotes

Lots of things in XML need to be in quotes. It's easy to miss an error like the missing quote in the code below:

```
<link name=”link_1></link>
```

It's also very easy to miss important details, such as :

## Joints need parents and children

```
<joint name=”my_joint” type=”fixed” >
<parent link=”link_1" />
</joint>
```

In the above example, although all the tags are matched, the joint does not have a child. This can result in strange errors such as "model has two root links", which means that the child link for `my_joint` hasn't been connected to the tree.

# Using XACRO 

To use XACRO in your XML file, you need to make sure that it starts with a robot tag that contains the `xmlns:xacro` string, such as this one: 

```
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="krytn" encoding='UTF-8'>
```

## Including files 

Once you have set that up, you can now use xacro to import or run macros. Importing a xacro file looks like this: 

```
  <xacro:include filename="$(find magni_description)/urdf/magni.urdf.xacro" />
```

Note that the xacro tags all start with `xacro`. Also, observe the `${find magni_description}` syntax. This `$` indicates a special signal for XACRO to do some work. In this case it will find the path of the `magni_description` package and replace `${find magni_description}` with it. 

This include directive is the same as copy and pasting that file into that location of the xacro file. In the case of the `magni.urdf.xacro` it is adding the links and joints of the magni robot. 

This is a super helpful way to program, as it will let you create modular robot builds, and save you from repeating yourself in code. 

## Running Macros 

You can create your own macros, the file: `src/bar_examples/magni_description/urdf/magni.urdf.xacro` has a good example: 

```
    <xacro:macro name="box_inertial_with_origin" params="x y z mass *origin">
        <inertial>
            <mass value="${mass}" />
            <xacro:insert_block name="origin" />
            <inertia ixx="${0.0833333 * mass * (y*y + z*z)}" ixy="0.0" ixz="0.0" iyy="${0.0833333 * mass * (x*x + z*z)}" iyz="0.0" izz="${0.0833333 * mass * (x*x + y*y)}" />
        </inertial>
    </xacro:macro>
```

This defines an inertial macro named `bax_inertial` with origin. This macro takes parameters `x`, `y` , `z`, 'mass' and also a whole `origin` tag. 

Whenever the macro is used, it replaces it with the contents of the macro. So for example, in `src/bar_examples/magni_description/urdf/magni.urdf.xacro`, it is called :

```
   <xacro:box_inertial_with_origin x="0.398" y="0.221" z="0.150" mass="10.0">
      <origin xyz="-0.15 0 0.03" rpy="0 0 0" />
    </xacro:box_inertial_with_origin>
```

And so the xacro will replace that code with: 

        <inertial>
            <mass value="${mass}" />
            <xacro:insert_block name="origin" />
            <inertia ixx="${0.0833333 * mass * (y*y + z*z)}" ixy="0.0" ixz="0.0" iyy="${0.0833333 * mass * (x*x + z*z)}" iyz="0.0" izz="${0.0833333 * mass * (x*x + y*y)}" />
        </inertial>

and then proceed to evaluate the variables  with the values: `x="0.398" y="0.221" z="0.150" mass="10.0"` and the origin tag: `<origin xyz="-0.15 0 0.03" rpy="0 0 0" />`

You'll notice some multiplication, this is because Xacro can also perform simple maths on variables. 
## Doing Maths 

You can also use xacro to do maths. For example, this snippet from `src/bar_examples/magni_description/urdf/magni.urdf.xacro`: 

```
 <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <collision>
        <origin xyz="${0.033*reflect} 0 0" rpy="0 ${pi/2} 0" />
        <geometry>
          <cylinder radius="${wheel_r}" length="0.01"/>
        </geometry>
      </collision>
```

Here you can see the use of the `$` to use the variables in the origin tags, but you can also use a placeholder for pi, e.g.

```
 rpy="0 ${pi/2} 0"
```

Here this code makes a rotation of pi/2 radians around the y axis. This can be much more readable than writing the equivalent in radians here, and shows the use of maths on the pi variable. 

## Conditionals 

You can even do conditional logic, like an if statement:

```
<xacro:if value="True">
<!-- do something -->
</xacro:if>
```

# Running Xacro 

Sometimes xacro fails, and you need to try and debug it. You can actually run the xacro parser from the command line. Just run: 

`xacro <filename>` and it will either output the full file, or it will give an error. You can also use `xacro -v <filename>` to get more error details. 

Finally, you can easily inspect an output from xacro by outputting it into a file, using the linux command: 

`xacro <filename>  > out.urdf`

for example 
```
xacro -v src/bar_examples/magni_description/urdf/magni.urdf.xacro > out.urdf
```

# Walkthrough 

Here is a video walking you through the features of XACRO and URDF.

https://youtu.be/Fyey923jWlM