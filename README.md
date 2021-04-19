# Emerging-Sys-Arch-Tech
CS-350: Emerging Sys Arch &amp; Tech

#### Summarize the project and what problem it was solving?
The final project consisted of creating a thermostate which allowed user input via UART to accept an intial set point. After initialization, the TI board would then activate a sensor to read the current temp within the room. If the temperature was greater or lower than the setpoint, the value would increment or decrement in order to reach the user specified set point. If the temp needed to increase, the TI board would have an LED turn on using GPIO to signify the temp was heating up, and would shut off when the temperature reached the setpoint. All of this is going on as concurrent tasks and is being written to the outpout terminal via UART.

#### What did you do particularly well?
I felt that my organizational approach to breaking apart pieces of the code to increase readability was my strong point. I am by no means an expert in this though, my primary experience in development comes from C# which provides a bit more flexibility that C does in terms of how things are structured. But I tried my best to keep things organized, even if that means keeping similar topics together. (IE: Keeping GPIO, I2C, Timer, UART code close together)

#### Where could you improve?
My biggest weakness is having proper documentation provided to accurately describe my work. I think in general it is good practice to create the documentation before the code implementation as to have a general understand of the logic being written. I spent a lot of time through trial and error which could have been avoided or lessened if I had taken the time to properly plan my course of action.

#### What tools and/or resources are you adding to your support network?
I've never worked with Code Composer Studio before so that is a new tool in my arsenal. As mentioned elsewhere, I've found a new insight working with product documentation that I previously didn't have which is a great resource.

#### What skills from this project will be particularly transferable to other projects and/or course work?
Reading the TI board's documentation and going to the decleration of header files proved to help me work through most problems. Often times it's very easy to rely on google, but when a problem is very specific, or if you need to find a thorough answer to how somethign on the board is implemented the documentation specs provided are a great resource. Up until this class, I had never really spent much time looking into product documentation.

#### How did you make this project maintainable, readable, and adaptable?
A general concept I noticed throughout the class and the Labs was to include a lot of the logic within the main thread. Throughout my time as a Software Developer, I've learned that breaking thing into smaller, more maintainble chunks helps in the long run with maintainability. I chose to break out the code based on it's functionality, in that I tried to keep the task setup together, I2C together, and so on for the other components used here. Doing this allowed me to quickly target the code and isolate points of concern when working through issues, whereas if I had kept most of the code in the main thread I could have spent a lot of time reading through the same code to find the problem point. This not only increases maintainability but helps in readability too.