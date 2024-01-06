# Stealth Robotics 4089
we like to code röböt
## Coding Guidelines
Here are some guidelines for writing code for our robot, for your reference when developing new systems for or reviers to cross-check during a pull request.
##### General:
- Code should follow general formatting rules. Class names are upper camel case, and method and variable names are lower camel case. Constants should be completely upper case, with underscores as separtors.
- Stealth has a standard of calling subsystems _______Subsystem, Commands _______Command, and commands intended to be default commands _______DefaultCommand. Please stick to this standard for consistency.
- Code should be formatted regularly to keep it readable (`Alt+Shift+F` on VSCode).
- Code should be commented when appropriate. 
##### Safety:
- When tuning a PID for the first time, clamp the motor output. You can still print and graph the actual calculation, but limiting motor power when still tuning will prevent unexpected and unsafe robot action.
- Systems should have current limits when appropriate. We dont want to have to burn fuses or trip breakers if we don't have to.
- When testing code, make others aware when the robot is enabled and what will happen when your code runs.
##### Regarding Subsystem and Command control schemes, and rules for making them not interfere with the other systems or cause damage to a robot:
- Any subsystem which controls a motor with a PID should
    - have the PID(s) contained within the subsystem
    - have a periodic to keep the PID(s) updated and adjust the motor power as necessary
    - only use commands to ask the subsystem to change its setpoint
    - No public “set power command”, which would override the PIDs control.
    - Rationale: this is likely the common case, where you want a subsystem to be able to maintain a position regardless of whether or not a default command (or any command!) is running. Plus, if you want to use a PID to control motor it’s usually quite wrong to let someone set its power directly.
- All commands declare their subsystem requirements
    - calling `addRequirements` with all subsystems in the constructor
    - using the multi-argument `InstantCommand` constructor which lists it’s requirements.
        - So `InstantCommand(() -> ASubsystem.foo())` is illegal. It would have to be `InstantCommand(() -> ASubsystem.foo(), ASubsystem)`
    - Rationale: the beauty of the command paradigm in WPILib is this notion that we can fire them off whenever we want and know they’ll interrupt whatever else was going on that matters. We can sequence them, put them in parallel, let them override default commands, etc. But that breaks down if you don’t declare subsystems properly, and it’s something we almost always forget to do with instant commands.
- Every command should be prepared to be interrupted, and do something with the boolean interrupted parameter in end()
    - Especially if the command is doing multi-step work, consider and note clearly whether or not it’s okay to, say, leave a motor running, or a far-away setpoint set, etc.
    - Rationale: again, to maintain the value of the command paradigm we have to allow for sensible interruption.
## Git Workflow
**Do not push to main**. If you want to add a new feature, create a **feature branch**. This can be done by going to Branches -> View All Branches -> Create a new branch. Clone main into the branch, and code whatever you need to code! Once it is tested and **ready to go on the robot**, and it is commented where necessary with readable formatting, create a pull request by going to Pull Requests -> Create Pull Request and select the branch you have been working on. Write out a **detailed** description of your changes and submit the pull request. One of the team's verified reviewers will go through and test your changes. They may leave comments and request changes to your code, and once all changes and comments are resolved, your changes will be merged into main and the branch will be deleted.

If you have submitted a pull request, and it has been a full meeting and the review process has not started on your pull request, feel free to message the reviewers on Slack!

Reviewers promise to always:
- Make sure to always respect the programmers. We understand that you do not have to write code exactly how we would, and we don't need to think or act like you.
- We will always only evaluate code based on how it will affect the robot. If the code would hurt the robot, hurt people, or cause harm to the codebase of the robot later on, then we will work together to get the code to a place where it is ready to be on main. Reviewers are only there to look for bugs, leaky or broken abstractions, egregious efficiency issues, and redundant functionality or duplicated code.
- We will not focus on things like extra parentheses, variable names that we don't like, or other petty things that don't actually matter. These will only be commented on if they are truly confusing and could cause other programmers on the team to slip up. 

Putting your code out there to review is hard. We know you spent time and energy making that code, and for many it can feel like a form of self expression. It is hard to do, and we acknowledge that. The reviewers will **always** be respectful and will only offer information for the benefit of your coding knowledge and the health of the robot. We look forward to coding with all of you!
