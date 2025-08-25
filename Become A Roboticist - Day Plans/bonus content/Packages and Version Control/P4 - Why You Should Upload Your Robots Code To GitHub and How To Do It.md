Once you've successfully created a ROS package, you're going to want to share it. 

The best practice way to share it is to put it into a Git repository. You can then use a tool like VCS (see video P3) to corral lots of repos into a workspace. 

Watch out, this video is full of details that can be easy to get wrong. I've found my students often struggle with SSH keys, although they are easy to make, they can also be easy to lose. And when they aren't available connecting to github will error in unclear ways. 

# Action Steps 

1. Run the command `ssh-keygen` to create an ssh key, don't add a passphrase, it will be super annoying if you do (trust me on this!). 
2. Run `cat ~/.ssh/id_rsa.pub` and copy the output to your clipboard, this is your public key that you can share with the world.
3. Go to GitHub, make an account, and add the contents of your public key as an SSH key in github. You're computer can now talk to github easily.
4. Find a repo that you want to clone, e.g. find_object_2d and click on the green button, select ssh and then copy the url into your command line after `git clone <url>`. Make sure you are in your `src` directory. 


## But why not just give my friend a USB key with the source code? 

Yes, you could just give your friend a USB with the code. But what if you want to push them a small bug fix? Will you meet up with them again? Perhaps you'll send them an email and let them know the line number that needs to be changed? What if they make an improvement and want to give it back to you, but you've already done extra work yourself, how can you combine these code bases?

If you use git with GitHub all of those challenges become very easy to deal with, the only hard part is getting your code up on GitHub in the first place. 

# Walkthrough Video

https://youtu.be/I01wthy8sko