#ifndef QUEUE_H
#define QUEUE_H

#define QUEUE_MAX_SIZE  256

struct queue
{
  int front;
  int rear;
  int size;
  unsigned char data[QUEUE_MAX_SIZE];
};


void initialize_queue(struct queue* queue);
int  enqueue_character(struct queue* queue, const unsigned char character);
int  dequeue_character(struct queue* queue, unsigned char* character);
int  enqueue_string(struct queue* queue, const unsigned char* string, int string_size);
int  get_queue_room(struct queue* queue);
int  get_queue_size(struct queue* queue);

#endif
