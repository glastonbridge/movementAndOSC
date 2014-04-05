

(
b = Bus.control(s,8);
c = Bus.control(s,1);

b.set(1,1,1,1,1,1,1,1,1); // for safety!
~oldnumbers = [0,0,0,0,0,0,0,0];

n = OSCFunc({|msg, time, addr, recvPort|
	var ident,numbers,moon;
	msg.postln;
	numbers = msg[2..];
	moon = msg[1];
	numbers.postln;

	b.set(numbers);
	c.set(moon);
	Synth(\starStruck);
}, '/tonz', nil, 57120);

SynthDef(\starStruck, {
	var env;
	env = Line.ar(1,0,1,doneAction:2);
	Out.ar([0,1],FreeVerb.ar(env*
		(SinOsc.ar(200+(200/In.kr(b,8))))))
}).load;

e= {Saw.ar(140) + Saw.ar(In.kr(c))*LFSaw.ar(1)*0.1}.play
e.free
)

