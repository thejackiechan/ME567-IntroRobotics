/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

  heap.push(new_element);
  new_element_location = heap.length - 1;

  while(heap[new_element_location] < heap[Math.floor((new_element_location - 1)/2)] && new_element_location != 0){
  parent_location = Math.floor((new_element_location - 1)/2); // cannot define this before while loop b/c parent_location can change
    
  temp = heap[parent_location]; // temp = parent value
  heap[parent_location] = heap[new_element_location]; // parent = new element value
  heap[new_element_location] = temp; // replace new element with parent value
  new_element_location = parent_location;
  }
  return heap;
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
      */

// define extract function for min binary heap
function minheap_extract(heap) {

  if(heap.length > 1){
    swap_location = 0;
    root_element = heap[swap_location];
    last_element = heap.pop();
    heap[swap_location] = last_element;

    while((heap[swap_location] > heap[(2*swap_location) + 1] && ((2*swap_location) + 1) < heap.length)
    || (heap[swap_location] > heap[(2*swap_location) + 2] && ((2*swap_location) + 2) < heap.length)){

      child1_location = (2*swap_location) + 1; // cannot define these locations before while loop because swap_location can change
      child2_location = (2*swap_location) + 2;

      if(child2_location < heap.length){
        if(heap[child1_location] < heap[child2_location]){
          priority = child1_location;
         }else{
          priority = child2_location;
       }
      }else{
        priority = child1_location; // for when child2 is not located in heap
      }
      temp = heap[priority];
      heap[priority] = heap[swap_location];
      heap[swap_location] = temp;
      swap_location = priority;
    }
  }else if(heap.length == 1){
    root_element = heap.pop();
  }
  return root_element;
}

// assign extract function within minheaper object

minheaper.extract = minheap_extract;






