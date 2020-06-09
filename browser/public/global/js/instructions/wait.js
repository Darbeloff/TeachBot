/**
 * Waits for specified event.
 *
 * Pause the module until one of the following happens, based on instr.for:
 * - 'total effort': The sum of all joint efforts of the robot are instr.to_be (e.g. '>') instr.val (e.g. 1.5).
 * - 'time': instr.s seconds have passed.
 * - 'custom': instr.function evaluates as true
 * - 'endpoint pos': TODO
 * - 'user input': TODO
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.wait = async function(instr, instructionAddr) {
	checkInstruction(instr, ['for'], instructionAddr);
	if (VERBOSE) console.log(`Waiting for ${instr.for}...`);

	return new Promise((resolve, reject) => {
		switch (instr.for) {
			case ('total effort'):
				checkInstruction(instr, ['to_be', 'val'], instructionAddr);

				var to_be = instr.to_be;
				var percent = eval(self.hashTokeyVal(instr.val))/100;
				var thresh = 0;
				for (let i=0; i<JOINTS; i++) {
						thresh += self.dictionary[`EFFORT_${i}`];
					}

				var val = thresh+thresh*percent

				self.dictionary['wait_output'] = false;

				function waitForTotalEffort() {
					var totalEffort = 0;
					for (let j=0; j<JOINTS; j++) {
						totalEffort += self.dictionary[`EFFORT_${j}`];
					}

					switch (to_be) {
						case ('<'):
							if (!(totalEffort<val)) {
								setTimeout(waitForTotalEffort, 100);
								return;
							} else { 
								if (instr.hasOwnProperty('store_answer_in')){
									self.dictionary['wait_output'] = true
								}
								resolve(); 
							}
							break;

						case ('<='):
							if (!(totalEffort<=val)) {
								setTimeout(waitForTotalEffort, 100);
								return;
							} else { resolve(); }
							break;

						case ('=='):
							if (!(totalEffort==val)) {
								setTimeout(waitForTotalEffort, 100);
								return;
							} else { resolve(); }
							break;

						case ('>='):
							if (!(totalEffort>=val)) {
								setTimeout(waitForTotalEffort, 100);
								return;
							} else { resolve(); }
							break;

						case ('>'):
							if (!(totalEffort>val)) {
								setTimeout(waitForTotalEffort, 100);
								return;
							} else { 
								if (instr.hasOwnProperty('store_answer_in')){
									self.dictionary['wait_output'] = true
								}
								resolve(); 
							}
							break;

						case ('!='):
							if (!(totalEffort!=val)) {
								setTimeout(waitForTotalEffort, 100);
								return;
							} else { resolve(); }
							break;

						default:
							throw `Cannot wait for total effort to be ${to_be} ${instr.val}.`
					}
				}

				waitForTotalEffort();
				break;

			case 'time':
				checkInstruction(instr, ['s'], instructionAddr);
				setTimeout(() => { resolve(); }, 1000*instr.s);
				break;

			case 'custom':
				checkInstruction(instr, ['function'], instructionAddr);

				this.button_topic.subscribe(async function(message) {
					console.log('changing dict value')
					var value = parseInt(message.data)
					self.dictionary['lastButton'] = value
					self.button_topic.unsubscribe();
					self.button_topic.removeAllListeners();
				});
				
				function waitForCustom() {
					if (!eval(self.hashTokeyVal(instr.function))) {
						setTimeout(waitForCustom, 100);
					} else { resolve(); }
				}

				waitForCustom();
				break;

			case 'endpoint pos':
				throw 'TODO: Add wait for endpoint pos functionality';

			case 'user input':
				this.button_topic.subscribe(async function(message) {
					console.log('here')
					self.button_topic.unsubscribe();
					self.button_topic.removeAllListeners();
					resolve();
				});

				break;

			default:
				throw `Wait for "${instr.for}" unsupported`;
		}
	});
};