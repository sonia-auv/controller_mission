This software have few limitations :

- Please ensure all state have a different name
- A concurrent state must not have a transition to a concurrent state. As a workaround, create a dummy state that will just forward transition to next concurrent state.
- 
