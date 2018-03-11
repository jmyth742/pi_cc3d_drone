// State function return codes
// Close for near wall
// Far for not near wall
// Fail for exceptional circumstances
enum return_codes { close, far, fail };

// State codes define a state
// Entry state for initialization
// Safe state for normal behaviour
// Danger state for evasive behaviour
// End state for shutting down and cleanup
enum state_codes { entry, safe, danger, end };

// Define a state transition
// Gives the mapping between source state, return code -> destination state
struct transition {
  enum state_codes srcState;
  enum return_codes retCode;
  enum state_codes dstState;
};

// Define the state transition table
// There are no transitions from end state
struct transition state_transitions[] = {
  { entry, close, danger },
  { entry, far, safe },
  { entry, fail, end },

  { safe, close, danger },
  { safe, far, safe },
  { safe, fail, end },

  { danger, close, danger },
  { danger, far, safe },
  { danger, fail, end },
};

// Declare the state function prototypes
int entry_state(void);
int safe_state(void);
int danger_state(void);
int end_state(void);

// Define a set of state function pointers
int (* state[])(void) = { entry_state, safe_state, danger_state, end_state };

// The program
int main() {
  // Start in the entry state
  enum state_codes currentState = entry;

  // Keep track of the last returned code and currently selected state function
  enum return_codes returnCode;
  int (* stateFun)(void);

  // While not in end state
  while (currentState != end) {
    // Get a pointer to the current states function
    stateFun = state[currentState];

    // Invoke state function and assign to return code
    returnCode = stateFun();

    // Given the return code and the current state lookup the new state
    currentState = lookup_transition(currentState, returnCode);
  }
}

// Declare the lookup state transition function prototype
enum state_codes(state_codes, return_codes);

// Define the lookup state transition function
enum state_codes(state_codes src, return_codes retCode) {
 
}
